// #include "stdafx.h"

#define RVLVN_TIME_MESUREMENT // Vidovic
#ifdef RVLVN_TIME_MESUREMENT
#define RVLVN_RECORD_RESULTS
#undef RVLVN_VERBOSE
#ifndef RVLLINUX
#include <Windows.h>
#endif
#endif

#include "RVLCore2.h"
#include <algorithm>
#include "RVLVTK.h"
#include <vtkTriangle.h>
#include <vtkVertexGlyphFilter.h>
#include "Util.h"
#include "Space3DGrid.h"
#include "Graph.h"
#include "Mesh.h"
#include "MSTree.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
#include "ReconstructionEval.h"
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
#include "ObjectDetector.h"
#ifdef RVLLINUX
#include <Eigen/Eigenvalues>
#else
#include <Eigen\Eigenvalues>
#endif
#include "NeighborhoodTool.h"
#include "VN.h"
#include "VNClass.h"
#include "VNInstance.h"
#include "VNClassifier.h"
#include "Gripper.h"
#include <ctime>

// #define RVLVN_CLASSIFY_CTI_VERSION0
#define RVLVN_CLASSIFY_CTI_VERSION1
#define RVLVN_CLASSIFY_CTI_VERSION1B
// #define RVLVN_CLASSIFY_HYPOTHESIS_CTI_LOG
#define RVLVN_PART_SEGMENTATION_TRAINING_LOG
#define RVL_MERGE_CLUSTERS_VERBOSE

using namespace RVL;
using namespace RECOG;

VNClassifier::VNClassifier()
{
    mode = RVLRECOGNITION_MODE_TRAINING;
    problem = RVLRECOGNITION_PROBLEM_CLASSIFICATION;
    dataSet = RVL_DATASET_FLAG_TUW_KINECT;
    trainingMethod = 0;
    recognitionMethod = 0;
    voxelSize = 0.01f;
    sampleVoxelDistance = 2;
    visualizationData.resolution = 0.01f;
    visualizationData.SDFSurfaceValue = 0.0f;
    visualizationData.bVisualizeConvexClusters = false;
    visualizationData.bVisualizeConcaveClusters = false;
    visualizationData.bVisualizeSurfels = false;
    visualizationData.bVisualizeAllClusters = false;
    visualizationData.bVisualizeMergedClusters = false;
    visualizationData.bVisualizeCTIs = false;
    visualizationData.bVisualizeZAxes = false;
    visualizationData.bVisualizeSceneInterpretation = false;
    visualizationData.bVisualizeVNInstance = false;
    modelDB = RVLVN_MODELDB_NONE;
    modelDataBase = NULL;    // Vidovic
    modelsInDataBase = NULL; // Vidovic
    scenesInDataBase = NULL;
    modelSequenceFileName = NULL;
    VNShapeSpaceFileName = NULL;
    ModelVNInstanceListFileName = NULL;
    componentFileName = NULL;
    modelLayerFileName = NULL;
    SceneVNInstanceListFileName = NULL;
    ScenePointCellID = NULL;
    PartSuperSegmentID = NULL;
    primitiveDataBase = NULL;
    sceneFileName = NULL;
    validFeaturesFileName = NULL;
    representativesFileName = NULL;
    pShapeInstanceDetection = NULL;
    pNeighborhoodTool = NULL;
    classArray.Element = NULL;
    classArray.n = 0;
    sceneObject.sampleArray.Element = NULL;
    refModel.d = NULL;
    refModel.bd = NULL;
    alignment.modelDataBase = NULL;
    sampledUnitSphere.Element = NULL;
    clusterMap = NULL;
    SClusters.Element = NULL;
    SO3Samples.Element = NULL;
    convexTemplateLUT.LUT.Element = NULL;
    CTIDescriptorMapingData.iD = NULL;
    CTIDescriptorMapingData.tree = NULL;
    CTIDescriptorMapingData.R = NULL;
    iSurfelArray.Element = NULL;
    iVertexArray.Element = NULL;
    iConcavitySurfelArray.Element = NULL;
    iConcavityVertexArray.Element = NULL;
    bSurfelInArray = NULL;
    bVertexInArray = NULL;
    convexHullVertices.Element = NULL;
    concavityVertices.Element = NULL;
    hypotheses.Element = NULL;
    superSegments.Element = NULL;
    superSegments.n = 0;
    interpretation.Element = NULL;
    sortedHypotheses.Element = NULL;
    nSurfelCellHypotheses = NULL;
    modelTemplates.Element = NULL;
    componentClusters.Element = NULL;
    sortedSuperSegmentHypotheses = NULL;
    sortedModelHypotheses = NULL;
    iObjectVertexArray.Element = NULL;
    fitData = NULL;
    D = NULL;
    bD = NULL;
    f = NULL;
    R = NULL;
    pObjects = NULL;
    primitiveLayerOutput.q = NULL;
    primitiveLayerOutput.y = NULL;
    primitiveLayerOutput.R = NULL;
    modelVNArray.Element = NULL;
    sceneVNArray.Element = NULL;
    modelSet.Element = NULL;
    firstComponentAbsIdx.Element = NULL;
    modelSegmentInterval = NULL;
    esComponent.Element = NULL;
    etComponent.Element = NULL;
    activeGauss = NULL;
    activeGaussMem = NULL;
    metamodelGaussAssociation = NULL;
    metamodelGaussAssociationCost = NULL;
    activeMetamodelGauss = NULL;
    activeMetamodelGaussMem = NULL;
    DM = NULL;
    bDM = NULL;
    modelCH = NULL;
    modelBoundingSphere = NULL;
    unassignedSurfels.Element = NULL;
    probabilisticAssociationWorkData.bInitialized = false;
    associationProbabilityWorkData.bInitialized = false;
    hypothesisEvaluationLevel3Method = 2;
    maxnSCClusters = 4;
    maxnSUClusters = 2;
    maxnSTClusters = 2;
    nHypothesesLevel3 = 30; // BASELINE
    connectedComponentMaxDist = 0.050f;
    connectedComponentMinSize = 100;
    maxDistFromNHull = 0.0f;
    maxedmax = 0.0f;
    wePosition = 1.0f;
    weSize = 1.0f;
    kMaxMatchCost = 0.06f;
    kTangentDistanceTolerance = 0.01f;
    tangentOrientationTolerance = 0.01f;
    maxBoundingSphereRadius = 0.2f;
    kZMin2 = 5.0f;
    alignmentSupportThr = 0.5f;
    maxOutlierPerc = 10; // %
    maxz = 2.0f;         // m
    collisionPercThr = 50;
    convexityErrorCoeff = 0.01f;
    invisibleComponentPenal = 0.0001f;
    unassignedPointCost = 0.0001f;
    multiplePointAssignmentCost = 0.0001f;
    outlierCost = 0.001f;
    sigmaRdeg = 10.0f; // deg
    lambda = 0.0f;
    stdTranslation = 1.0;
    stdSize = 0.5;
    stdShape = 0.1;
    kComponentUncert = 0.04;
    priorProbabilityComponent = 1e-6;
    beta = 0.01;
    gama = 0.01;
    dUnassignedPointsLogProbability = 5.0f;
    dUnassignedPoints = 0.1f;
    CTIMatchSigmad1 = 0.02f;
    CTIMatchSigmad2 = 0.2f;
    CTIMatchRedundantPoseOrientThr = 0.7f;
    ICPSuperSegmentRadius = 0.1f;
    hypothesisEvaluationLevel3CHTolerance = 0.03f;
    simulatedAnnealingnIterationsPerHypothesis = 50;
    simulatedAnnealingnSamples = 500;
    kFaceClusterSizeThr = 0.04f;
    // minSuperSegmentSegmentSize = 300;
    fitParams.alpha = 20.0f;
    fitParams.beta = 0.02f;
    fitParams.lambda0 = 0.0f;
    fitParams.lambda1 = 0.01f;
    fitParams.regularization = 3;
    fitParams.maxe = 0.010f;
    fitParams.bInit = false;
    fitParams.bGnd = true;
    fitParams.bGnd2 = true;
    fitParams.maxnIterations = 20;
    fitParams.nFitRotLSIterations = 10;
    b3DNetTestSet = false;
    b3DNetVOI = false;
    bBoundingBoxes = false;
    bDetectSupportingSurfaces = false;
    bGndConstraint = true;
    bVisualization = false;
    bVisualizeBestInstance = false;
    bLoadCTIDataBase = true;
    bConcavity = true;
    bLoadModelVNInstances = false;
    bLoadSceneVNInstances = false;
    bSaveGeneratedVNInstances = false;
    bSaveSupersegments = false;
    bVNCTIAlignment = false;
    bSingleClass = false;
    bGenerateModelCTIs = false;
    bCreateAlignmentTree = false;
    bPrimitiveLayer = false;
    bCTIAlignment = false;
    bTrainingStructure = false;
    bDepthImage = true;
    bOwnsObjectGraph = false;
    bAlignWithConvexTemplate = false;
    bHorizontalBottomAssumption = false;
    bShapeSpace = false;
    bPlusPlus = false;
    bInstanceMatchWithLabels = true;
    bMST = false;
    bCAG = false;
    bGenerateModelVNInstances = true;
    bGenerateSceneVNInstances = true;

    bSampleModelPointClouds = true;
    bConcaveSuperSegments = true;
    bUseColor = false;
    bUseLevel3MatchedPercentageThr = true;
    bPartSegmentationMetamodelClusters = true;
    level3MatchedPercentageThr = 0.80f;
    greedyInterpretationMatchedPercentageThr = 0.25f;
    greedyInterpretationColorThr = 0.25f;
    edThr = 0.6f;
    subsampleModelsVoxelSize = 8.0f;
    nIterationsComponentAssociationLocalSearch = 1000;
    nSamplesComponentAssociationLocalSearch = 100;
    metamodelChangeProbability = 0.1f;
    labelChangeProbability = 0.1f;
    componentClusterMaxCost = 1.0f;
    componentAssociationAlphat = 1.0f;
    componentAssociationAlphas = 1.0f;
    componentAssociationBeta = 1.0f;
    componentAssociationGamma = 0.999f;
    componentAssociationSig = 0.02f;
    alphaCTINetPrior = 1.0f;
    componentNeighborhoodMatchUnmatchPenal = 0.1f;
    wModelSimilarity = 1.0f;
    wNeighborhoodSimilarity = 0.2f;
    componentAssociationfCompThr = 0.05f;

    hypClass = new int[10];
    memset(hypClass, 0, 10 * sizeof(int));

    descriptorMem.Create(1000000000);

    QList<VN> *pVNList = &VNList;

    RVLQLIST_INIT(pVNList);

    timeInterpret = 0;

    debug1 = debug2 = -1;

    // Vidovic - for printing vertices to file
    iConvexCluster = 0;
    iConcaveCluster = 0;

    bMergeClusters = true;             // Vidovic
    proximityThresh = 0.2;             // Vidovic
    CHDistanceThresh = 0.05;           // Vidovic
    mergeThresh1 = 0.05;               // Vidovic
    mergeThresh2 = 0.2;                // Vidovic
    bSaveClusterDistanceFile = false;  // Vidovic
    bDistanceToCHUsingNormals = false; // Vidovic
    trainingModelScale = 1.0f;
    iSelectedClass = -1;
    iRefModel = iRefComponent = 0;
    pTimer = NULL;

    nTP = 0;
    nFP = 0;
    nFN = 0;

    maxGTPartLabel = -1;

    bVisualizePartAssociation = false;
    bLabelsAssignedToUnassignedPoints = false;

    varTranslation = stdTranslation * stdTranslation;
    varSize = stdSize * stdSize;
    varShape = stdShape * stdShape;
}

VNClassifier::~VNClassifier()
{
    Clear();
}

void VNClassifier::Create(char *cfgFileName)
{
    // sampledUnitSphere <- set of unit vectors obtained by uniform sampling of the unit sphere

    alignment.TemplateMatrix(sampledUnitSphere);

    // Convex template size.

    nCT = sampledUnitSphere.h;

    // Sample SO3.

    SampleSO3();

    // Create convex clustering tool.

    convexClustering.pMem = pMem;
    convexClustering.pMem0 = pMem0;

    convexClustering.pSurfels = pSurfels;

    convexClustering.pSurfelDetector = pSurfelDetector;

    convexClustering.CreateParamList(convexClustering.pMem0);
    convexClustering.ParamList.LoadParams(cfgFileName);

    convexClustering.bDetectGroundPlane = false;
    convexClustering.bOverlappingClusters = true;
    convexClustering.bEdgeClusters = true;
    convexClustering.bObjectDetection = false;

    // Create concave clustering tool.

    concaveClustering.pMem = pMem;
    concaveClustering.pMem0 = pMem0;

    concaveClustering.pSurfels = pSurfels;

    concaveClustering.pSurfelDetector = pSurfelDetector;

    concaveClustering.CreateParamList(concaveClustering.pMem0);
    concaveClustering.ParamList.LoadParams(cfgFileName);

    concaveClustering.clusterType = -1.0f;

    concaveClustering.bDetectGroundPlane = false;
    concaveClustering.bOverlappingClusters = true;
    concaveClustering.bEdgeClusters = true;
    concaveClustering.bObjectDetection = false;

    // Create plane detection tool.

    planeDetector.pMem = pMem;
    planeDetector.pMem0 = pMem0;

    planeDetector.pSurfels = pSurfels;

    planeDetector.pSurfelDetector = pSurfelDetector;

    planeDetector.CreateParamList(planeDetector.pMem0);
    planeDetector.ParamList.LoadParams(cfgFileName);

    planeDetector.clusterType = 0.0f;

    planeDetector.bDetectGroundPlane = false;
    planeDetector.bOverlappingClusters = false;
    planeDetector.bEdgeClusters = true;
    planeDetector.bObjectDetection = false;

    /// Create meta models.

    char metaModelFileName[] = "metamodel0.dat";

    FILE *fp;

    VN *pModel;

    // Create meta model RVLVN_METAMODEL_CONVEX.

    pModel = new VN;

    VN_::CreateConvex(pModel, alignment.convexTemplate66, pMem0);

    // sprintf(metaModelFileName + strlen(metaModelFileName) - 5, "%1d.dat", RVLVN_METAMODEL_CONVEX);

    // fp = fopen(metaModelFileName, "w");

    // pModel->SaveFeatures(fp);

    // fclose(fp);

    // models.push_back(pModel);
    metaModels.push_back(pModel);

    // Create meta model RVLVN_METAMODEL_BANANA.

    pModel = new VN;

    VN_::CreateBanana(pModel, alignment.convexTemplate66, pMem0);

    // sprintf(metaModelFileName + strlen(metaModelFileName) - 5, "%1d.dat", RVLVN_METAMODEL_BANANA);

    // fp = fopen(metaModelFileName, "w");

    // pModel->SaveFeatures(fp);

    // fclose(fp);

    // models.push_back(pModel);
    metaModels.push_back(pModel);

    // Create meta model RVLVN_METAMODEL_BOTTLE.

    pModel = new VN;

    // VN_::CreateBottle(pModel, alignment.convexTemplate66, pMem0);
    VN_::CreateTwoConvex(pModel, alignment.convexTemplate66, pMem0);

    // sprintf(metaModelFileName + strlen(metaModelFileName) - 5, "%1d.dat", RVLVN_METAMODEL_BOTTLE);

    // fp = fopen(metaModelFileName, "w");

    // pModel->SaveFeatures(fp);

    // fclose(fp);

    // models.push_back(pModel);
    metaModels.push_back(pModel);

    // Create meta model RVLVN_METAMODEL_BOWL.

    pModel = new VN;

    VN_::CreateBowl2(pModel, alignment.convexTemplate66, pMem0);

    // sprintf(metaModelFileName + strlen(metaModelFileName) - 5, "%1d.dat", RVLVN_METAMODEL_BOWL);

    // fp = fopen(metaModelFileName, "w");

    // pModel->SaveFeatures(fp);

    // fclose(fp);

    // models.push_back(pModel);
    metaModels.push_back(pModel);

    // Create meta model RVLVN_METAMODEL_TORUS.

    pModel = new VN;

    VN_::CreateTorus(pModel, alignment.convexTemplate66, pMem0);

    // sprintf(metaModelFileName + strlen(metaModelFileName) - 5, "%1d.dat", RVLVN_METAMODEL_TORUS);

    // fp = fopen(metaModelFileName, "w");

    // pModel->SaveFeatures(fp);

    // fclose(fp);

    // models.push_back(pModel);
    metaModels.push_back(pModel);

    // Create meta model RVLVN_METAMODEL_HAMMER.

    pModel = new VN;

    VN_::CreateHammer(pModel, alignment.convexTemplate66, pMem0);

    // sprintf(metaModelFileName + strlen(metaModelFileName) - 5, "%1d.dat", RVLVN_METAMODEL_HAMMER);

    // fp = fopen(metaModelFileName, "w");

    // pModel->SaveFeatures(fp);

    // fclose(fp);

    // models.push_back(pModel);
    metaModels.push_back(pModel);

    // Create meta model RVLVN_METAMODEL_MUG.

    pModel = new VN;

    // VN_::CreateMug(pModel, pMem0);
    // VN_::CreateMug2(pModel, alignment.convexTemplate66, pMem0);
    // VN_::CreateMug3(pModel, alignment.convexTemplate66, pMem0);
    VN_::CreateMug4(pModel, alignment.convexTemplate66, pMem0);

    // sprintf(metaModelFileName + strlen(metaModelFileName) - 5, "%1d.dat", RVLVN_METAMODEL_MUG);

    // fp = fopen(metaModelFileName, "w");

    // pModel->SaveFeatures(fp);

    // fclose(fp);

    // models.push_back(pModel);
    metaModels.push_back(pModel);

    // Create meta model RVLVN_METAMODEL_PIPE.

    pModel = new VN;

    VN_::CreatePipe(pModel, alignment.convexTemplate66, pMem0);
    // VN_::CreateBowl2(pModel, alignment.convexTemplate66, pMem0);

    // sprintf(metaModelFileName + strlen(metaModelFileName) - 5, "%1d.dat", RVLVN_METAMODEL_PIPE);

    // fp = fopen(metaModelFileName, "w");

    // pModel->SaveFeatures(fp);

    // fclose(fp);

    // models.push_back(pModel);
    metaModels.push_back(pModel);

    // Create meta model RVLVN_METAMODEL_CAR.

    // pModel = new VN;

    // VN_::CreateCar(pModel, alignment.convexTemplate66, pMem0);

    // sprintf(metaModelFileName + strlen(metaModelFileName) - 5, "%1d.dat", RVLVN_METAMODEL_CAR);

    // fp = fopen(metaModelFileName, "w");

    // pModel->SaveFeatures(fp);

    // fclose(fp);

    // models.push_back(pModel);

    // Create meta model RVLVN_METAMODEL_APPLE.

    // pModel = new VN;

    // VN_::CreateApple(pModel, alignment.convexTemplate66, pMem0);

    // sprintf(metaModelFileName + strlen(metaModelFileName) - 5, "%1d.dat", RVLVN_METAMODEL_APPLE);

    // fp = fopen(metaModelFileName, "w");

    // pModel->SaveFeatures(fp);

    // fclose(fp);

    // models.push_back(pModel);

    // Create meta model RVLVN_METAMODEL_MUG5.

    pModel = new VN;

    VN_::CreateMug5(pModel, alignment.convexTemplate66, pMem0);
    metaModels.push_back(pModel);

    // Set parameters.

    CreateParamList();

    paramList.LoadParams(cfgFileName);

    sigmaR = sigmaRdeg * DEG2RAD;

    clusteringTolerance = 3.0f * convexClustering.kNoise * 2.0f / pSurfelDetector->kPlane;

    varTranslation = stdTranslation * stdTranslation;
    varSize = stdSize * stdSize;
    varShape = stdShape * stdShape;

    if (b3DNetTestSet)
        b3DNetVOI = true;

    if (problem == RVLRECOGNITION_PROBLEM_CLASSIFICATION || problem == RVLRECOGNITION_PROBLEM_PART_SEGMENTATION)
    {
        // Create object classes.

        if (modelDB == RVLVN_MODELDB_3DNET)
            VN_::_3DNetDatabaseClasses(this);
        else if (modelDB == RVLVN_MODELDB_OSD)
            VN_::OSDDatabaseClasses(this);
        else if (modelDB == RVLVN_MODELDB_SHAPENET)
            VN_::ShapeNetDatabaseClasses(this);
        else
        {
            classArray.n = 2;

            classArray.Element = new ClassData[classArray.n];
        }

        // Load shape space.

        if (VNShapeSpaceFileName)
            LoadShapeSpace(VNShapeSpaceFileName);
    }

    // Configure

    if (problem == RVLRECOGNITION_PROBLEM_PART_SEGMENTATION)
    {
        convexClustering.bLabelConstrainedClustering = concaveClustering.bLabelConstrainedClustering = true;

        maxnSCClusters = maxnSUClusters = -1;

        if (bLoadModelVNInstances)
            LoadVNInstances(this, ModelVNInstanceListFileName, &modelVNList, &modelVNArray);

        FILE *fpModels = fopen(modelLayerFileName, "rb");

        // LoadPCs();

        if (fpModels == NULL)
        {
            if (!bGenerateModelVNInstances)
                LoadPCs();
        }
        else
            fclose(fpModels);

        VNClass objClass;

        classes.push_back(objClass);

        if (mode == RVLRECOGNITION_MODE_RECOGNITION)
        {
            char *VNPartModelDataBaseFileName = RVLCreateFileName(modelDataBase, ".dat", -1, ".vn.part.dat");

            FILE *fpVNDB = fopen(VNPartModelDataBaseFileName, "rb");

            delete[] VNPartModelDataBaseFileName;

            classes[0].Load(this, fpVNDB);

            fclose(fpVNDB);

            VNInstance VNInstanceTemplate;

            VNInstanceTemplate.AssociationProbabilityComputationInit(this, 0, &associationProbabilityWorkData);
            VNInstanceTemplate.ProbabilisticAssociationInit(this, 0, &probabilisticAssociationWorkData);

            bMST = false;
        }
    }

    //

    if (problem == RVLRECOGNITION_PROBLEM_SHAPE_INSTANCE_DETECTION && mode == RVLRECOGNITION_MODE_RECOGNITION)
    {
        bConcaveSuperSegments = false;

        if (pShapeInstanceDetection == NULL)
            pShapeInstanceDetection = new PSGM;

        pShapeInstanceDetection->pMem = pMem;
        pShapeInstanceDetection->pMem0 = pMem0;

        pShapeInstanceDetection->pSurfels = pSurfels;

        pShapeInstanceDetection->pSurfelDetector = pSurfelDetector;

        pShapeInstanceDetection->CreateParamList(convexClustering.pMem0);
        pShapeInstanceDetection->ParamList.LoadParams(cfgFileName);

        pShapeInstanceDetection->LoadModelDataBase();

        pShapeInstanceDetection->ModelClusters(modelSegmentInterval, maxnModelSegments);

        pShapeInstanceDetection->LoadModelMeshDB(modelSequenceFileName, &(pShapeInstanceDetection->vtkModelDB), false, 0.4);

        pShapeInstanceDetection->vtkRMSEModelDB = pShapeInstanceDetection->vtkModelDB;

        DM = new float[pShapeInstanceDetection->MCTISet.pCTI.n * sampledUnitSphere.h];
        float *dM = DM;
        bDM = new uchar[pShapeInstanceDetection->MCTISet.pCTI.n * sampledUnitSphere.h];
        uchar *bdM = bDM;

        int i, iMCTI;
        PSGM_::ModelInstance *pMCTI;

        for (iMCTI = 0; iMCTI < pShapeInstanceDetection->MCTISet.pCTI.n; iMCTI++)
        {
            pMCTI = pShapeInstanceDetection->MCTISet.pCTI.Element[iMCTI];

            for (i = 0; i < sampledUnitSphere.h; i++)
            {
                *(dM++) = pMCTI->modelInstance.Element[i].d;
                *(bdM++) = (uchar)(pMCTI->modelInstance.Element[i].valid);
            }
        }

        CreateConvexHullsAndBoundingSpheres();

        // Visualizer visualizer;

        // visualizer.Create();

        // pShapeInstanceDetection->SubsampleModels(modelSequenceFileName, 0.1f, (pShapeInstanceDetection->bModelsInMillimeters ? 0.001f : 1.0f));
        // pShapeInstanceDetection->SubsampleModels(modelSequenceFileName, 1000, &visualizer);
        // pShapeInstanceDetection->SubsampleModels2(modelSequenceFileName, 8.0f, 0.001f); //BASELINE
        pShapeInstanceDetection->SubsampleModels2(modelSequenceFileName, subsampleModelsVoxelSize, 0.001f); // Experiment 7
        // pShapeInstanceDetection->SubsampleModels2(modelSequenceFileName, 10.0f, 0.001f); //Experiment 8
        // pShapeInstanceDetection->SubsampleModels2(modelSequenceFileName, 8.0f, 0.001f, &visualizer);
    }

    if (problem == RVLRECOGNITION_PROBLEM_TORUS_DETECTION)
        maxnSCClusters = maxnSUClusters = -1;

    // Initialize alignment tool.

    if (mode == RVLRECOGNITION_MODE_TRAINING)
    {
        bDepthImage = false;

        if (trainingMethod != 4 && problem != RVLRECOGNITION_PROBLEM_PART_SEGMENTATION)
            bCTIAlignment = true;
    }

    if (bCTIAlignment || b3DNetTestSet)
    {
        alignment.pMem0 = pMem0;
        alignment.pMem = pMem;
        alignment.pSurfels = pSurfels;
        alignment.pSurfelDetector = pSurfelDetector;

        alignment.problem = RVLRECOGNITION_PROBLEM_CLASSIFICATION;

        alignment.CreateParamList(alignment.pMem0);
        alignment.ParamList.LoadParams(cfgFileName);

        alignment.Init(cfgFileName);

        alignment.pObjects->objectArray.n = 0;

        pObjects = alignment.pObjects;

        if (bGenerateModelCTIs)
            bLoadCTIDataBase = false;

        if (modelDB != RVLVN_MODELDB_NONE)
        {
            alignment.modelDataBase = RVLCreateString(modelDataBase);

            if (bLoadCTIDataBase)
            {
                printf("Loading CTI database %s...", modelDataBase);

                alignment.LoadModelDataBase();

                printf("completed.\n");
            }
        }

        alignment.bGroundPlaneRFDescriptors = true;
    }
    else if (pObjects == NULL)
    {
        pObjects = new SURFEL::ObjectGraph;

        pObjects->CreateParamList(pMem0);

        pObjects->ParamList.LoadParams(cfgFileName);

        pObjects->pMem = pMem;
        pObjects->pSurfels = pSurfels;

        bOwnsObjectGraph = true;
    }

    // Create convex template lookup table.

    printf("Creating convex template lookup table...");

    CreateTemplateLookUpTable(sampledUnitSphere, 200, convexTemplateLUT);

    printf("completed.\n");

    // Load CTI descriptor mapper data.

    PSGM_::LoadCTIDescriptorMapping("..\\CTIDescriptorMapper.dat", CTIDescriptorMapingData);

    // Allocate memory for classification.

    int iClass;
    RECOG::ClassData *pClass;

    if (bShapeSpace)
    {
        fitData = new RECOG::VN_::FitData[classArray.n];

        for (iClass = 0; iClass < classArray.n; iClass++)
        {
            pClass = classArray.Element + iClass;

            RECOG::VN_::Init(fitData[iClass], pClass->M, fitParams.beta, fitParams.lambda0, fitParams.lambda1, false, 0);
        }
    }

    int nZ = CTIDescriptorMapingData.nPan * (CTIDescriptorMapingData.nTilt - 1) + 1;

    int nDescriptors = nZ * 6 * 4 * CTIDescriptorMapingData.nRoll;

    D = new float[sampledUnitSphere.h * nDescriptors];

    bD = new uchar[sampledUnitSphere.h * nDescriptors];

    f = new float[sampledUnitSphere.h * nDescriptors];

    R = new float[9 * nZ * CTIDescriptorMapingData.nRoll];

    // Initialize primitive layer output.

    if (classArray.n > 0)
    {
        primitiveLayerOutput.mTotal = 0;

        for (iClass = 0; iClass < classArray.n; iClass++)
        {
            pClass = classArray.Element + iClass;

            primitiveLayerOutput.mTotal += pClass->M.h;
        }

        primitiveLayerOutput.R = new float[9 * alignment.CTIDescMap.nCTI];
        primitiveLayerOutput.q = new float[primitiveLayerOutput.mTotal * alignment.CTIDescMap.nCTI];
        primitiveLayerOutput.y = new float[classArray.n * alignment.CTIDescMap.nCTI];
        primitiveLayerOutput.JR = new float[3 * alignment.CTIDescMap.nCTI];
    }
}

void VNClassifier::CreateParamList()
{
    paramList.m_pMem = pMem0;

    RVLPARAM_DATA *pParamData;

    paramList.Init();

    pParamData = paramList.AddParam("VN.trainingMethod", RVLPARAM_TYPE_INT, &trainingMethod);
    pParamData = paramList.AddParam("VN.++", RVLPARAM_TYPE_BOOL, &bPlusPlus);
    pParamData = paramList.AddParam("VN.recognitionMethod", RVLPARAM_TYPE_INT, &recognitionMethod);
    pParamData = paramList.AddParam("VN.3DNetTestSet", RVLPARAM_TYPE_BOOL, &b3DNetTestSet);
    pParamData = paramList.AddParam("VN.3DNetVOI", RVLPARAM_TYPE_BOOL, &b3DNetVOI);
    pParamData = paramList.AddParam("VN.BoundingBoxes", RVLPARAM_TYPE_BOOL, &bBoundingBoxes);
    pParamData = paramList.AddParam("VN.detectSupportingSurfaces", RVLPARAM_TYPE_BOOL, &bDetectSupportingSurfaces);
    pParamData = paramList.AddParam("VN.ModelDB", RVLPARAM_TYPE_ID, &modelDB);
    paramList.AddID(pParamData, "3DNET", RVLVN_MODELDB_3DNET);
    paramList.AddID(pParamData, "OSD", RVLVN_MODELDB_OSD);
    paramList.AddID(pParamData, "SHAPENET", RVLVN_MODELDB_SHAPENET);
    pParamData = paramList.AddParam("VN.GenerateModelCTIs", RVLPARAM_TYPE_BOOL, &bGenerateModelCTIs);
    pParamData = paramList.AddParam("VN.bCreateAlignmentTree", RVLPARAM_TYPE_BOOL, &bCreateAlignmentTree);
    pParamData = paramList.AddParam("VN.primitiveLayer", RVLPARAM_TYPE_BOOL, &bPrimitiveLayer);
    pParamData = paramList.AddParam("VN.wePosition", RVLPARAM_TYPE_FLOAT, &wePosition);
    pParamData = paramList.AddParam("VN.weSize", RVLPARAM_TYPE_FLOAT, &weSize);
    pParamData = paramList.AddParam("VN.kMaxMatchCost", RVLPARAM_TYPE_FLOAT, &kMaxMatchCost);
    pParamData = paramList.AddParam("VN.tangentOrientationTolerance", RVLPARAM_TYPE_FLOAT, &tangentOrientationTolerance);
    pParamData = paramList.AddParam("VN.kTangentDistanceTolerance", RVLPARAM_TYPE_FLOAT, &kTangentDistanceTolerance);
    pParamData = paramList.AddParam("VN.maxnSCClusters", RVLPARAM_TYPE_INT, &maxnSCClusters);
    pParamData = paramList.AddParam("VN.maxnSUClusters", RVLPARAM_TYPE_INT, &maxnSUClusters);
    pParamData = paramList.AddParam("VN.maxnSTClusters", RVLPARAM_TYPE_INT, &maxnSTClusters);
    pParamData = paramList.AddParam("Recognition.problem", RVLPARAM_TYPE_ID, &problem);
    paramList.AddID(pParamData, "CLASSIFICATION", RVLRECOGNITION_PROBLEM_CLASSIFICATION);
    paramList.AddID(pParamData, "PART_SEGMENTATION", RVLRECOGNITION_PROBLEM_PART_SEGMENTATION);
    paramList.AddID(pParamData, "SHAPE_INSTANCE_DETECTION", RVLRECOGNITION_PROBLEM_SHAPE_INSTANCE_DETECTION);
    paramList.AddID(pParamData, "TORUS_DETECTION", RVLRECOGNITION_PROBLEM_TORUS_DETECTION);
    pParamData = paramList.AddParam("Recognition.mode", RVLPARAM_TYPE_ID, &mode);
    paramList.AddID(pParamData, "TRAINING", RVLRECOGNITION_MODE_TRAINING);
    paramList.AddID(pParamData, "RECOGNITION", RVLRECOGNITION_MODE_RECOGNITION);
    pParamData = paramList.AddParam("ModelDataBase", RVLPARAM_TYPE_STRING, &modelDataBase);                 // Vidovic
    pParamData = paramList.AddParam("ModelsInDataBase", RVLPARAM_TYPE_STRING, &modelsInDataBase);           // Vidovic
    pParamData = paramList.AddParam("ScenesInDataBase", RVLPARAM_TYPE_STRING, &scenesInDataBase);           // Vidovic
    pParamData = paramList.AddParam("ModelSequenceFileName", RVLPARAM_TYPE_STRING, &modelSequenceFileName); // VIDOVIC
    pParamData = paramList.AddParam("VN.PrimitiveCTIDataBase", RVLPARAM_TYPE_STRING, &primitiveDataBase);
    pParamData = paramList.AddParam("VN.voxelSize", RVLPARAM_TYPE_FLOAT, &voxelSize);
    pParamData = paramList.AddParam("VN.connectedComponentMaxDist", RVLPARAM_TYPE_FLOAT, &connectedComponentMaxDist);
    pParamData = paramList.AddParam("VN.connectedComponentMinSize", RVLPARAM_TYPE_INT, &connectedComponentMinSize);
    pParamData = paramList.AddParam("VN.maxDistFromNHull", RVLPARAM_TYPE_FLOAT, &maxDistFromNHull);
    pParamData = paramList.AddParam("VN.createDescriptor.maxedValid", RVLPARAM_TYPE_FLOAT, &maxedmax);
    pParamData = paramList.AddParam("VN.fit.alpha", RVLPARAM_TYPE_FLOAT, &(fitParams.alpha));
    pParamData = paramList.AddParam("VN.fit.beta", RVLPARAM_TYPE_FLOAT, &(fitParams.beta));
    pParamData = paramList.AddParam("VN.fit.lambda0", RVLPARAM_TYPE_FLOAT, &(fitParams.lambda0));
    pParamData = paramList.AddParam("VN.fit.lambda1", RVLPARAM_TYPE_FLOAT, &(fitParams.lambda1));
    pParamData = paramList.AddParam("VN.fit.regularization", RVLPARAM_TYPE_INT, &(fitParams.regularization));
    pParamData = paramList.AddParam("VN.fit.maxe", RVLPARAM_TYPE_FLOAT, &(fitParams.maxe));
    pParamData = paramList.AddParam("VN.fit.kSceneSupport", RVLPARAM_TYPE_FLOAT, &(fitParams.kSceneSupport));
    pParamData = paramList.AddParam("VN.fit.kOutliers", RVLPARAM_TYPE_FLOAT, &(fitParams.kOutliers));
    pParamData = paramList.AddParam("VN.fit.kHull", RVLPARAM_TYPE_FLOAT, &(fitParams.kHull));
    pParamData = paramList.AddParam("VN.fit.kZMin", RVLPARAM_TYPE_FLOAT, &(fitParams.kZMin));
    pParamData = paramList.AddParam("VN.fit.cosSurfaceRayAngleThr", RVLPARAM_TYPE_FLOAT, &(fitParams.cosSurfaceRayAngleThr));
    pParamData = paramList.AddParam("VN.fit.maxnIterations", RVLPARAM_TYPE_INT, &(fitParams.maxnIterations));
    pParamData = paramList.AddParam("VN.fit.nFitRotLSIterations", RVLPARAM_TYPE_INT, &(fitParams.nFitRotLSIterations));
    pParamData = paramList.AddParam("VN.fit.bGndConstraint2", RVLPARAM_TYPE_BOOL, &(fitParams.bGnd2));
    pParamData = paramList.AddParam("VN.alignWithConvexTemplate", RVLPARAM_TYPE_BOOL, &bAlignWithConvexTemplate);
    pParamData = paramList.AddParam("VN.horizontalBottomAssumption", RVLPARAM_TYPE_BOOL, &bHorizontalBottomAssumption);
    pParamData = paramList.AddParam("VN.maxBoundingSphereRadius", RVLPARAM_TYPE_FLOAT, &maxBoundingSphereRadius);
    pParamData = paramList.AddParam("VN.kZMin2", RVLPARAM_TYPE_FLOAT, &kZMin2);
    pParamData = paramList.AddParam("VN.alignmentSupportThr", RVLPARAM_TYPE_FLOAT, &alignmentSupportThr);
    pParamData = paramList.AddParam("VN.maxOutlierPerc", RVLPARAM_TYPE_INT, &maxOutlierPerc);
    pParamData = paramList.AddParam("VN.maxz", RVLPARAM_TYPE_FLOAT, &maxz);
    pParamData = paramList.AddParam("VN.collisionPercThr", RVLPARAM_TYPE_INT, &collisionPercThr);
    pParamData = paramList.AddParam("VN.convexityErrorCoeff", RVLPARAM_TYPE_FLOAT, &convexityErrorCoeff);
    pParamData = paramList.AddParam("VN.invisibleComponentPenal", RVLPARAM_TYPE_FLOAT, &invisibleComponentPenal);
    pParamData = paramList.AddParam("VN.unassignedPointCost", RVLPARAM_TYPE_FLOAT, &unassignedPointCost);
    pParamData = paramList.AddParam("VN.multiplePointAssignmentCost", RVLPARAM_TYPE_FLOAT, &multiplePointAssignmentCost);
    pParamData = paramList.AddParam("VN.outlierCost", RVLPARAM_TYPE_FLOAT, &outlierCost);
    pParamData = paramList.AddParam("VN.sigmaRdeg", RVLPARAM_TYPE_FLOAT, &sigmaRdeg);
    pParamData = paramList.AddParam("VN.CTIMatch.sigmad1", RVLPARAM_TYPE_FLOAT, &CTIMatchSigmad1);
    pParamData = paramList.AddParam("VN.CTIMatch.sigmad2", RVLPARAM_TYPE_FLOAT, &CTIMatchSigmad2);
    pParamData = paramList.AddParam("VN.CTIMatch.redundantPoseOrientThr", RVLPARAM_TYPE_FLOAT, &CTIMatchRedundantPoseOrientThr);
    pParamData = paramList.AddParam("VN.ICP.SuperSegmentRadius", RVLPARAM_TYPE_FLOAT, &ICPSuperSegmentRadius);
    pParamData = paramList.AddParam("VN.hypothesisEvaluationLevel3.CHTolerance", RVLPARAM_TYPE_FLOAT, &hypothesisEvaluationLevel3CHTolerance);
    pParamData = paramList.AddParam("VN.simulatedAnnealing.nIterationsPerHypothesis", RVLPARAM_TYPE_INT, &simulatedAnnealingnIterationsPerHypothesis);
    pParamData = paramList.AddParam("VN.simulatedAnnealing.nSamples", RVLPARAM_TYPE_INT, &simulatedAnnealingnSamples);
    pParamData = paramList.AddParam("VN.kFaceClusterSizeThr", RVLPARAM_TYPE_FLOAT, &kFaceClusterSizeThr);
    pParamData = paramList.AddParam("VN.visualization", RVLPARAM_TYPE_BOOL, &bVisualization);
    pParamData = paramList.AddParam("VN.visualizeBestInstance", RVLPARAM_TYPE_BOOL, &bVisualizeBestInstance);
    pParamData = paramList.AddParam("VN.concavities", RVLPARAM_TYPE_BOOL, &bConcavity);
    pParamData = paramList.AddParam("VN.bGndConstraint", RVLPARAM_TYPE_BOOL, &bGndConstraint);
    pParamData = paramList.AddParam("VN.singleClass", RVLPARAM_TYPE_BOOL, &bSingleClass);
    pParamData = paramList.AddParam("VN.visualizeConvexClusters", RVLPARAM_TYPE_BOOL, &(visualizationData.bVisualizeConvexClusters));
    pParamData = paramList.AddParam("VN.visualizeConcaveClusters", RVLPARAM_TYPE_BOOL, &(visualizationData.bVisualizeConcaveClusters));
    pParamData = paramList.AddParam("VN.visualizeSurfels", RVLPARAM_TYPE_BOOL, &(visualizationData.bVisualizeSurfels));
    pParamData = paramList.AddParam("VN.visualizeMergedClusters", RVLPARAM_TYPE_BOOL, &(visualizationData.bVisualizeMergedClusters));
    pParamData = paramList.AddParam("VN.visualizeAllClusters", RVLPARAM_TYPE_BOOL, &(visualizationData.bVisualizeAllClusters));
    pParamData = paramList.AddParam("VN.visualizeCTIs", RVLPARAM_TYPE_BOOL, &(visualizationData.bVisualizeCTIs));
    pParamData = paramList.AddParam("VN.visualizeSceneFitting", RVLPARAM_TYPE_BOOL, &(visualizationData.bVisualizeSceneFitting));
    pParamData = paramList.AddParam("VN.visualizeZAxes", RVLPARAM_TYPE_BOOL, &(visualizationData.bVisualizeZAxes));
    pParamData = paramList.AddParam("VN.visualizeSceneInterpretation", RVLPARAM_TYPE_BOOL, &(visualizationData.bVisualizeSceneInterpretation));
    pParamData = paramList.AddParam("VN.visualizeVNInstance", RVLPARAM_TYPE_BOOL, &(visualizationData.bVisualizeVNInstance));
    pParamData = paramList.AddParam("VN.ShapeSpaceFileName", RVLPARAM_TYPE_STRING, &VNShapeSpaceFileName);
    pParamData = paramList.AddParam("VN.ValidFeaturesFileName", RVLPARAM_TYPE_STRING, &validFeaturesFileName);
    pParamData = paramList.AddParam("VN.ModelVNInstanceListFileName", RVLPARAM_TYPE_STRING, &ModelVNInstanceListFileName);
    pParamData = paramList.AddParam("VN.SceneVNInstanceListFileName", RVLPARAM_TYPE_STRING, &SceneVNInstanceListFileName);
    pParamData = paramList.AddParam("VN.ComponentFileName", RVLPARAM_TYPE_STRING, &componentFileName);
    pParamData = paramList.AddParam("VN.ScenePointCellID", RVLPARAM_TYPE_STRING, &ScenePointCellID);
    pParamData = paramList.AddParam("VN.PartSuperSegmentID", RVLPARAM_TYPE_STRING, &PartSuperSegmentID);
    pParamData = paramList.AddParam("VN.ModelLayerFileName", RVLPARAM_TYPE_STRING, &modelLayerFileName);
    pParamData = paramList.AddParam("VN.RepresentativesFileName", RVLPARAM_TYPE_STRING, &representativesFileName);
    pParamData = paramList.AddParam("VN.trainingStructure", RVLPARAM_TYPE_BOOL, &bTrainingStructure);
    pParamData = paramList.AddParam("VN.debug1", RVLPARAM_TYPE_INT, &debug1);
    pParamData = paramList.AddParam("VN.debug2", RVLPARAM_TYPE_INT, &debug2);
    pParamData = paramList.AddParam("VN.LearnMetamodels", RVLPARAM_TYPE_BOOL, &bLearnMetamodels);
    pParamData = paramList.AddParam("VN.GenerateModelVNInstances", RVLPARAM_TYPE_BOOL, &bGenerateModelVNInstances);
    pParamData = paramList.AddParam("VN.GenerateSceneVNInstances", RVLPARAM_TYPE_BOOL, &bGenerateSceneVNInstances);
    pParamData = paramList.AddParam("VN.SampleModelPointClouds", RVLPARAM_TYPE_BOOL, &bSampleModelPointClouds);
    pParamData = paramList.AddParam("VN.MST", RVLPARAM_TYPE_BOOL, &bMST);
    pParamData = paramList.AddParam("VN.CAG", RVLPARAM_TYPE_BOOL, &bCAG);
    pParamData = paramList.AddParam("VN.LoadModelVNInstances", RVLPARAM_TYPE_BOOL, &bLoadModelVNInstances);
    pParamData = paramList.AddParam("VN.LoadSceneVNInstances", RVLPARAM_TYPE_BOOL, &bLoadSceneVNInstances);
    pParamData = paramList.AddParam("VN.SaveGeneratedVNInstances", RVLPARAM_TYPE_BOOL, &bSaveGeneratedVNInstances);
    pParamData = paramList.AddParam("VN.SaveSupersegments", RVLPARAM_TYPE_BOOL, &bSaveSupersegments);
    pParamData = paramList.AddParam("VN.Alignment", RVLPARAM_TYPE_BOOL, &bVNCTIAlignment);
    pParamData = paramList.AddParam("VN.proximityThresh", RVLPARAM_TYPE_FLOAT, &proximityThresh);   // Vidovic
    pParamData = paramList.AddParam("VN.CHDistanceThresh", RVLPARAM_TYPE_FLOAT, &CHDistanceThresh); // Vidovic
    pParamData = paramList.AddParam("VN.mergeThresh1", RVLPARAM_TYPE_FLOAT, &mergeThresh1);         // Vidovic
    pParamData = paramList.AddParam("VN.mergeThresh2", RVLPARAM_TYPE_FLOAT, &mergeThresh2);         // Vidovic
    pParamData = paramList.AddParam("VN.lambda", RVLPARAM_TYPE_FLOAT, &lambda);
    pParamData = paramList.AddParam("VN.stdTranslation", RVLPARAM_TYPE_FLOAT, &stdTranslation);
    pParamData = paramList.AddParam("VN.stdSize", RVLPARAM_TYPE_FLOAT, &stdSize);
    pParamData = paramList.AddParam("VN.stdShape", RVLPARAM_TYPE_FLOAT, &stdShape);
    pParamData = paramList.AddParam("VN.kComponentUncert", RVLPARAM_TYPE_FLOAT, &kComponentUncert);
    pParamData = paramList.AddParam("VN.priorProbabilityComponent", RVLPARAM_TYPE_FLOAT, &priorProbabilityComponent);
    pParamData = paramList.AddParam("VN.dUnassignedPointsLogProbability", RVLPARAM_TYPE_FLOAT, &dUnassignedPointsLogProbability);
    pParamData = paramList.AddParam("VN.dUnassignedPoints", RVLPARAM_TYPE_FLOAT, &dUnassignedPoints);
    pParamData = paramList.AddParam("VN.mergeClusters", RVLPARAM_TYPE_BOOL, &bMergeClusters); // Vidovic
    pParamData = paramList.AddParam("VN.trainingModelScale", RVLPARAM_TYPE_FLOAT, &trainingModelScale);
    pParamData = paramList.AddParam("VN.SaveClusterDistanceFile", RVLPARAM_TYPE_BOOL, &bSaveClusterDistanceFile);   // Vidovic
    pParamData = paramList.AddParam("VN.DistanceToCHUsingNormals", RVLPARAM_TYPE_BOOL, &bDistanceToCHUsingNormals); // Vidovic
    pParamData = paramList.AddParam("Recognition.dataSet", RVLPARAM_TYPE_ID, &dataSet);                             // Vidovic
    paramList.AddID(pParamData, "TUW_KINECT", RVL_DATASET_FLAG_TUW_KINECT);                                         // Vidovic
    paramList.AddID(pParamData, "WILLOW_AND_CHALLENGE", RVL_DATASET_FLAG_WILLOW_AND_CHALLENGE);                     // Vidovic
    paramList.AddID(pParamData, "ICL", RVL_DATASET_FLAG_ICL);                                                       // Vidovic
    pParamData = paramList.AddParam("VN.hypothesisEvaluationLevel3Method", RVLPARAM_TYPE_INT, &hypothesisEvaluationLevel3Method);
    pParamData = paramList.AddParam("Recognition.useColor", RVLPARAM_TYPE_BOOL, &bUseColor); // Vidovic
    pParamData = paramList.AddParam("VN.useLevel3MatchedPercentageThr", RVLPARAM_TYPE_BOOL, &bUseLevel3MatchedPercentageThr);
    pParamData = paramList.AddParam("VN.level3MatchedPercentageThr", RVLPARAM_TYPE_FLOAT, &level3MatchedPercentageThr);
    pParamData = paramList.AddParam("VN.greedyInterpretationMatchedPercentageThr", RVLPARAM_TYPE_FLOAT, &greedyInterpretationMatchedPercentageThr);
    pParamData = paramList.AddParam("VN.greedyInterpretationColorThr", RVLPARAM_TYPE_FLOAT, &greedyInterpretationColorThr);
    pParamData = paramList.AddParam("VN.edThr", RVLPARAM_TYPE_FLOAT, &edThr);
    pParamData = paramList.AddParam("VN.nHypothesesLevel3", RVLPARAM_TYPE_INT, &nHypothesesLevel3);
    pParamData = paramList.AddParam("VN.subsampleModelsVoxelSize", RVLPARAM_TYPE_FLOAT, &subsampleModelsVoxelSize);
    pParamData = paramList.AddParam("VN.componentAssociationLocalSearch.nIterations", RVLPARAM_TYPE_INT, &nIterationsComponentAssociationLocalSearch);
    pParamData = paramList.AddParam("VN.componentAssociationLocalSearch.nSamples", RVLPARAM_TYPE_INT, &nSamplesComponentAssociationLocalSearch);
    pParamData = paramList.AddParam("VN.componentAssociationLocalSearch.metamodelChangeProbability", RVLPARAM_TYPE_FLOAT, &metamodelChangeProbability);
    pParamData = paramList.AddParam("VN.visualizePartAssociation", RVLPARAM_TYPE_BOOL, &bVisualizePartAssociation);
    pParamData = paramList.AddParam("VN.componentAssociationLocalSearch.labelChangeProbability", RVLPARAM_TYPE_FLOAT, &labelChangeProbability);
    pParamData = paramList.AddParam("VN.componentClusterMaxCost", RVLPARAM_TYPE_FLOAT, &componentClusterMaxCost);
    pParamData = paramList.AddParam("VN.ComponentAssociation.alphat", RVLPARAM_TYPE_FLOAT, &componentAssociationAlphat);
    pParamData = paramList.AddParam("VN.ComponentAssociation.alphas", RVLPARAM_TYPE_FLOAT, &componentAssociationAlphas);
    pParamData = paramList.AddParam("VN.ComponentAssociation.beta", RVLPARAM_TYPE_FLOAT, &componentAssociationBeta);
    pParamData = paramList.AddParam("VN.ComponentAssociation.gamma", RVLPARAM_TYPE_FLOAT, &componentAssociationGamma);
    pParamData = paramList.AddParam("VN.ComponentAssociation.sig", RVLPARAM_TYPE_FLOAT, &componentAssociationSig);
    pParamData = paramList.AddParam("VN.ComponentAssociation.neighborhoodMatchUnmatchPenal", RVLPARAM_TYPE_FLOAT, &componentNeighborhoodMatchUnmatchPenal);
    pParamData = paramList.AddParam("VN.ComponentAssociation.wModelSimilarity", RVLPARAM_TYPE_FLOAT, &wModelSimilarity);
    pParamData = paramList.AddParam("VN.ComponentAssociation.wNeighborhoodSimilarity", RVLPARAM_TYPE_FLOAT, &wNeighborhoodSimilarity);
    pParamData = paramList.AddParam("VN.ComponentAssociation.fCompThr", RVLPARAM_TYPE_FLOAT, &componentAssociationfCompThr);
    pParamData = paramList.AddParam("VN.refModel", RVLPARAM_TYPE_INT, &iRefModel);
    pParamData = paramList.AddParam("VN.refComponent", RVLPARAM_TYPE_INT, &iRefComponent);
}

void VNClassifier::Init(Mesh *pMesh)
{
    iSurfelArray.Element = new int[pSurfels->NodeArray.n];
    iVertexArray.Element = new int[pSurfels->vertexArray.n];
    iConcavitySurfelArray.Element = new int[pSurfels->NodeArray.n];
    iConcavityVertexArray.Element = new int[pSurfels->vertexArray.n];
    bSurfelInArray = new bool[pSurfels->NodeArray.n];
    memset(bSurfelInArray, 0, pSurfels->NodeArray.n * sizeof(bool));
    bVertexInArray = new bool[pSurfels->vertexArray.n];
    memset(bVertexInArray, 0, pSurfels->vertexArray.n * sizeof(bool));

    convexHullVertices.w = 3;

    convexHullVertices.Element = new float[3 * pSurfels->vertexArray.n];

    concavityVertices.w = 3;

    concavityVertices.Element = new float[3 * pSurfels->vertexArray.n];

    memConvexHull.Create(10000000);

    CTIMeshVertexMemSize = 10 * sampledUnitSphere.h;

    CTIMesh.NodeArray.Element = new Point[CTIMeshVertexMemSize];

    memCTIMesh.Create(10000000);

    convexTemplate = convexClustering.ConvexTemplatenT();

    QList<VN> *pVNList = &VNList;

    RVLQLIST_INIT(pVNList);
}

void VNClassifier::FreeTmpMem()
{
    delete[] iSurfelArray.Element;
    delete[] iVertexArray.Element;
    delete[] iConcavitySurfelArray.Element;
    delete[] iConcavityVertexArray.Element;
    delete[] bSurfelInArray;
    delete[] bVertexInArray;
    delete[] convexHullVertices.Element;
    delete[] concavityVertices.Element;

    VN *pVN = VNList.pFirst;

    VN *pNextVN;

    while (pVN)
    {
        pNextVN = pVN->pNext;

        delete pVN;

        pVN = pNextVN;
    }
}

void VNClassifier::Clear()
{
    int iModel;

    for (iModel = 0; iModel < models.size(); iModel++)
        delete[] models[iModel];

    models.clear();

    int iClass;
    ClassData *pClass;

    for (iClass = 0; iClass < classArray.n; iClass++)
    {
        pClass = classArray.Element + iClass;

        RVL_DELETE_ARRAY(pClass->M.Element);
        RVL_DELETE_ARRAY(pClass->qMin);
        RVL_DELETE_ARRAY(pClass->qMax);
        RVL_DELETE_ARRAY(pClass->A);
        RVL_DELETE_ARRAY(pClass->b);
        RVL_DELETE_ARRAY(pClass->q0);
        RVL_DELETE_ARRAY(pClass->valid);
        RVL_DELETE_ARRAY(pClass->VNGraspNormals_.Element);
        RVL_DELETE_ARRAY(pClass->faces.Element);
        RVL_DELETE_ARRAY(pClass->Q);
    }

    RVL_DELETE_ARRAY(classArray.Element);
    RVL_DELETE_ARRAY(sceneObject.sampleArray.Element);
    RVL_DELETE_ARRAY(refModel.d);
    RVL_DELETE_ARRAY(refModel.bd);
    RVL_DELETE_ARRAY(modelDataBase);
    RVL_DELETE_ARRAY(primitiveDataBase);
    RVL_DELETE_ARRAY(alignment.modelDataBase);
    RVL_DELETE_ARRAY(sceneFileName);
    RVL_DELETE_ARRAY(validFeaturesFileName);
    RVL_DELETE_ARRAY(sampledUnitSphere.Element);
    RVL_DELETE_ARRAY(clusterMap);
    RVL_DELETE_ARRAY(SClusters.Element);
    RVL_DELETE_ARRAY(SO3Samples.Element);
    RVL_DELETE_ARRAY(VNShapeSpaceFileName);
    RVL_DELETE_ARRAY(ModelVNInstanceListFileName);
    RVL_DELETE_ARRAY(SceneVNInstanceListFileName);
    RVL_DELETE_ARRAY(componentFileName);
    RVL_DELETE_ARRAY(modelLayerFileName);
    RVL_DELETE_ARRAY(ScenePointCellID);
    RVL_DELETE_ARRAY(PartSuperSegmentID);
    RVL_DELETE_ARRAY(modelSequenceFileName);
    RVL_DELETE_ARRAY(representativesFileName);
    RVL_DELETE_ARRAY(convexTemplateLUT.LUT.Element);
    PSGM_::DeleteCTIDescriptorMapping<float>(CTIDescriptorMapingData);
    RVL_DELETE_ARRAY(interpretation.Element);
    RVL_DELETE_ARRAY(sortedHypotheses.Element);
    RVL_DELETE_ARRAY(hypotheses.Element);
    RVL_DELETE_ARRAY(superSegments.Element);
    RVL_DELETE_ARRAY(nSurfelCellHypotheses);
    RVL_DELETE_ARRAY(modelTemplates.Element);
    RVL_DELETE_ARRAY(componentClusters.Element);
    RVL_DELETE_ARRAY(sortedSuperSegmentHypotheses);
    RVL_DELETE_ARRAY(sortedModelHypotheses);
    if (fitData)
    {
        int iClass;
        for (iClass = 0; iClass < classArray.n; iClass++)
            RECOG::VN_::Delete(fitData[iClass]);
        delete[] fitData;
    }
    RVL_DELETE_ARRAY(D);
    RVL_DELETE_ARRAY(bD);
    RVL_DELETE_ARRAY(f);
    RVL_DELETE_ARRAY(R);

    if (bOwnsObjectGraph)
        delete pObjects;

    RVL_DELETE_ARRAY(primitiveLayerOutput.q);
    RVL_DELETE_ARRAY(primitiveLayerOutput.y);
    RVL_DELETE_ARRAY(primitiveLayerOutput.R);
    RVL_DELETE_ARRAY(modelVNArray.Element);
    RVL_DELETE_ARRAY(sceneVNArray.Element);
    RVL_DELETE_ARRAY(modelSet.Element);
    RVL_DELETE_ARRAY(firstComponentAbsIdx.Element);
    RVL_DELETE_ARRAY(modelSegmentInterval);
    RVL_DELETE_ARRAY(esComponent.Element);
    RVL_DELETE_ARRAY(etComponent.Element);
    RVL_DELETE_ARRAY(activeGauss);
    RVL_DELETE_ARRAY(activeGaussMem);
    RVL_DELETE_ARRAY(metamodelGaussAssociation);
    RVL_DELETE_ARRAY(metamodelGaussAssociationCost);
    RVL_DELETE_ARRAY(activeMetamodelGauss);
    RVL_DELETE_ARRAY(activeMetamodelGaussMem);
    RVL_DELETE_ARRAY(DM);
    RVL_DELETE_ARRAY(bDM);
    if (pShapeInstanceDetection)
        for (iModel = 0; iModel < pShapeInstanceDetection->nModels; iModel++)
            delete modelCH[iModel];
    RVL_DELETE_ARRAY(modelCH);
    RVL_DELETE_ARRAY(modelBoundingSphere);
    RVL_DELETE_ARRAY(unassignedSurfels.Element);
    VNInstance VNInstanceTemp;

    VNInstanceTemp.AssociationProbabilityComputationFree(&associationProbabilityWorkData);
    VNInstanceTemp.ProbabilisticAssociationFree(&probabilisticAssociationWorkData);

    if (pShapeInstanceDetection)
        delete pShapeInstanceDetection;

    if (pTimer)
        delete pTimer;
}

void VNClassifier::ComputeDescriptor(
    Mesh *pMesh,
    float *RIn,
    float *tIn,
    float *&dS,
    bool *&bdS,
    Box<float> &SBoundingBox,
    int iModel,
    CRVLMem *pMem0,
    VNInstance **ppVNModel)
{
    // Detect surfels.

    pSurfels->Init(pMesh);

    pSurfelDetector->Init(pMesh, pSurfels, pMem);

    printf("Segmentation to surfels...");

    pSurfelDetector->Segment(pMesh, pSurfels);

    printf("completed.\n");

    int nSurfels = pSurfels->NodeArray.n;

    printf("No. of surfels = %d\n", nSurfels);

    pSurfels->DetectVertices(pMesh);

    // Create scene object.

    float *R = sceneObject.R;

    if (RIn)
    {
        RVLCOPYMX3X3(RIn, R);
    }
    else
    {
        RVLUNITMX3(R);
    }

    float *t = sceneObject.t;

    if (tIn)
    {
        RVLCOPY3VECTOR(tIn, t);
    }
    else
    {
        RVLNULL3VECTOR(t);
    }

    sceneObject.vertexArray = new float[3 * pSurfels->vertexArray.n];

    float *P = sceneObject.vertexArray;

    int iVertex;
    SURFEL::Vertex *pVertex;

    for (iVertex = 0; iVertex < pSurfels->vertexArray.n; iVertex++, P += 3)
    {
        pVertex = pSurfels->vertexArray.Element[iVertex];

        RVLTRANSF3(pVertex->P, R, t, P);
    }

    sceneObject.NArray = new float[3 * pSurfels->NodeArray.n];

    float *N = sceneObject.NArray;

    int iSurfel;
    Surfel *pSurfel;

    for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++, N += 3)
    {
        pSurfel = pSurfels->NodeArray.Element + iSurfel;

        RVLMULMX3X3VECT(R, pSurfel->N, N);
    }

    int nSamplePts = 300;

    pSurfels->SampleMeshDistanceFunction(pMesh, R, t, nSamplePts, voxelSize, sampleVoxelDistance, sceneObject.sampleArray);

    // Cluster surfels into convex and concave surfaces.

    bool bConcavity, bTorus;

    VN *pModel = (iModel >= 0 ? models[iModel] : NULL);

    if (trainingMethod == 0)
        pModel->ClusterTypes(bConcavity, bTorus);
    else if (trainingMethod == 1 || trainingMethod == 3 || trainingMethod == 4)
    {
        bConcavity = true;
        bTorus = false;
    }

    int nSCClusters, nSUClusters;

    RVL_DELETE_ARRAY(clusterMap)

    clusterMap = new int[pSurfels->NodeArray.n];

    RVL_DELETE_ARRAY(SClusters.Element);

    PSGM_::ConvexAndConcaveClusters(pMesh, pSurfels, pSurfelDetector, &convexClustering, &concaveClustering,
                                    SClusters, nSCClusters, nSUClusters, convexClustering.minClusterSize, clusterMap, maxnSCClusters, maxnSUClusters, bConcavity, (bTorus ? maxnSTClusters : 0),
                                    visualizationData.bVisualizeConvexClusters, visualizationData.bVisualizeConcaveClusters, visualizationData.bVisualizeSurfels, visualizationData.bVisualizeAllClusters);

    printf("No. of clusters = %d\n", SClusters.n);

    // fpDebug = fopen("debug.txt", "w");

    // fprintf(fpDebug, "nFeatures=%d\n", pModel->featureArray.n);

    // for (int i = 0; i < pModel->featureArray.n; i++)
    //	fprintf(fpDebug, "%f\t%f\t%f\n", pModel->featureArray.Element[i].N[0], pModel->featureArray.Element[i].N[1], pModel->featureArray.Element[i].N[2]);

    // for (int i = pModel->featureArray.n; i < pModel->NodeArray.n; i++)
    //	fprintf(fpDebug, "O%d\n", pModel->NodeArray.Element[i].operation);

    // RECOG::VN_::Edge *pEdge = pModel->EdgeList.pFirst;

    // while (pEdge)
    //{
    //	fprintf(fpDebug, "E%d-%d\n", pEdge->data.a, pEdge->data.b);

    //	pEdge = pEdge->pNext;
    //}

    // fprintf(fpDebug, "#conv.clusters %d, #concave clusters: %d\n", convexClustering.clusters.n, concaveClustering.clusters.n);

    // RECOG::PSGM_::Cluster *pSCluster_;

    // for (int i = 0; i < SClusters.n; i++)
    //{
    //	RECOG::SceneCluster *pSCluster = SClusters.Element + i;

    //	if (pSCluster->type == RVLVN_CLUSTER_TYPE_CONVEX || pSCluster->type == RVLVN_CLUSTER_TYPE_CONCAVE)
    //	{
    //		pSCluster_ = (RECOG::PSGM_::Cluster *)(pSCluster->vpCluster);

    //		fprintf(fpDebug, "%d\n", pSCluster_->size);
    //	}
    //}

    // Cluster surfels into convex surfaces.

    // convexClustering.pMesh = pMesh;

    // convexClustering.Clusters();

    // Cluster surfels into concave surfaces.

    // concaveClustering.pMesh = pMesh;

    // concaveClustering.Clusters();

    // Selection color for visualization.

    // uchar SelectionColor[] = {0, 255, 0};

    // Visualization

    // Visualizer visualizer;

    // visualizer.Create();

    // pSurfels->NodeColors(SelectionColor);

    // Visualization of convex clusters.

    // RVL_DELETE_ARRAY(convexClustering.clusterColor);

    // RandomColors(SelectionColor, convexClustering.clusterColor, convexClustering.clusters.n);

    // convexClustering.InitDisplay(&visualizer, pMesh, SelectionColor);

    // convexClustering.Display();

    // visualizer.Run();

    // pSurfels->DisplayData.bCallbackFunctionsDefined = false;

    // Visualization of concave clusters.

    // RVL_DELETE_ARRAY(concaveClustering.clusterColor);

    // RandomColors(SelectionColor, concaveClustering.clusterColor, concaveClustering.clusters.n);

    // concaveClustering.InitDisplay(&visualizer, pMesh, SelectionColor);

    // concaveClustering.Display();

    // visualizer.Run();

    // pSurfels->DisplayData.bCallbackFunctionsDefined = false;

    // Surfel visualization.

    // surfels.NodeColors(SelectionColor);

    // surfels.InitDisplay(&visualizer, &mesh, &surfelDetector);

    // surfels.Display(&visualizer, &mesh);

    // visualizer.Run();

    // pSurfels->DisplayData.bCallbackFunctionsDefined = false;

    if (trainingMethod == 0)
    {
        printf("Matching VN model to scene...");

        P = sceneObject.vertexArray;

        InitBoundingBox<float>(&SBoundingBox, P);

        P += 3;

        for (iVertex = 1; iVertex < pSurfels->vertexArray.n; iVertex++, P += 3)
            UpdateBoundingBox<float>(&SBoundingBox, P);

        dS = new float[pModel->featureArray.n];

        bdS = new bool[pModel->featureArray.n];

        // pModel->fpDebug = fpDebug;

        pModel->Match4(pMesh, sceneObject, this, SClusters, SBoundingBox, dS, bdS, bTrainingStructure);

        // fclose(fpDebug);

        delete[] sceneObject.vertexArray;
        delete[] sceneObject.NArray;

        printf("completed.\n");
    }
    else if (trainingMethod == 1)
    {
        // Vidovi�
        FILE *fpDescriptors = fopen("C:\\RVL\\ExpRez\\descriptors.txt", "a");
        FILE *fpDescriptorsMerged = fopen("C:\\RVL\\ExpRez\\descriptorsMerged.txt", "w");
        fclose(fpDescriptorsMerged);
        fpDescriptorsMerged = fopen("C:\\RVL\\ExpRez\\descriptorsMerged.txt", "a");
        // FILE *fpVertices = fopen("C:\\RVL\\ExpRez\\vertices.txt", "a");
        FILE *fpConvexVertices = fopen("C:\\RVL\\ExpRez\\toilet_paper_convex_vertices.txt", "a");
        FILE *fpConcaveVertices = fopen("C:\\RVL\\ExpRez\\toilet_paper_concave_vertices.txt", "a");
        int iMetaModel = 0;
        // int iModel = 0;
        // VN *pModel_ = models[iModel];

        RECOG::SceneCluster *pSCluster;
        RECOG::PSGM_::Cluster *pSCluster_;

        float R[9];
        RVLUNITMX3(R);

        // NHull is not used
        Array<SURFEL::NormalHullElement> NHull; // not used
        NHull.Element = NULL;
        NHull.n = 0;

        // float *dS_ = new float[pModel_->featureArray.n]; //change 66 to variable
        // uchar *bdS_ = new uchar[pModel_->featureArray.n]; //change 66 to variable
        float *dS_ = new float[SClusters.n * sampledUnitSphere.h];
        uchar *bdS_ = new uchar[SClusters.n * sampledUnitSphere.h];

        // 3.10.2018. - added because ComputeDescriptor function was changed
        float *NS = new float[sampledUnitSphere.h * sampledUnitSphere.w];
        int *iV = new int[sampledUnitSphere.h];

        int iCluster, partID;

        for (iCluster = 0; iCluster < SClusters.n; iCluster++)
        {
            pSCluster = SClusters.Element + iCluster;

            if (pSCluster->type == RVLVN_CLUSTER_TYPE_CONVEX || pSCluster->type == RVLVN_CLUSTER_TYPE_CONCAVE)
            {
                pSCluster_ = (RECOG::PSGM_::Cluster *)(pSCluster->vpCluster);

                partID = (pMesh->bLabels ? pSurfels->NodeArray.Element[pSCluster_->iSurfelArray.Element[0]].ObjectID : 0);

                pSCluster_->d = dS_ + sampledUnitSphere.h * iCluster;
                pSCluster_->bd = bdS_ + sampledUnitSphere.h * iCluster;

                if (pSCluster->type == RVLVN_CLUSTER_TYPE_CONVEX)
                {
                    // ComputeDescriptor(pSCluster_->iVertexArray, NHull, R, dS_, bdS_, 1.0F, true, false);
                    // SaveDescriptor(fpDescriptors, dS_, bdS_, 0, iMetaModel);
                    ComputeDescriptor(pSCluster_->iVertexArray, NHull, R, pSCluster_->d, pSCluster_->bd, NS, iV, 1.0F, true, false);
                    // SaveDescriptor(fpDescriptors, dS_, (bool*)bdS_, 0, iMetaModel);
                    SaveDescriptor(fpDescriptors, pSCluster_->d, pSCluster_->bd, R, sceneID, partID, iCluster, 0);

                    SaveVertices(fpConvexVertices, pSCluster_, iCluster, RVLVN_CLUSTER_TYPE_CONVEX);
                    // count primitives from 0
                    iConvexCluster++;
                }
                else
                {
                    // ComputeDescriptor(pSCluster_->iVertexArray, NHull, R, dS_, bdS_, -1.0F, true, false);
                    // SaveDescriptor(fpDescriptors, dS_, bdS_, 1, iMetaModel);
                    ComputeDescriptor(pSCluster_->iVertexArray, NHull, R, pSCluster_->d, pSCluster_->bd, NS, iV, -1.0F, true, false);
                    // SaveDescriptor(fpDescriptors, dS_, (bool*)bdS_, 1, iMetaModel);
                    SaveDescriptor(fpDescriptors, pSCluster_->d, pSCluster_->bd, R, sceneID, partID, iCluster, 1);

                    SaveVertices(fpConcaveVertices, pSCluster_, iCluster, RVLVN_CLUSTER_TYPE_CONCAVE);
                    // count primitives from 0
                    iConcaveCluster++;
                }
            }

            // SaveVertices(fpVertices, pSCluster_);
        }

        // Merge scene clusters
        bool verbose = false;

#ifdef RVL_MERGE_CLUSTERS_VERBOSE
        verbose = true;
#endif
        MergeClusters(pMesh, &SClusters, bMergeClusters, verbose);

        // Save descriptors from object array
        SURFEL::Object *pObject;
        int clusterType;

        for (int iObject = 0; iObject < pObjects->objectArray.n; iObject++)
        {
            pObject = pObjects->objectArray.Element + iObject;

            if (RVL::CheckFlag(pObject->flags, RVLPCSEGMENT_OBJECT_FLAG_CONCAVE))
                clusterType = 1;
            else
                clusterType = 0;

            SaveDescriptor(fpDescriptorsMerged, pObject->d, pObject->bd, R, sceneID, pObject->label, iObject, clusterType);
        }

        if (visualizationData.bVisualizeMergedClusters)
        {
            Visualizer visualizer;

            visualizer.Create();

            uchar selectionColor[] = {0, 255, 0};

            pObjects->InitDisplay(&visualizer, pMesh, selectionColor);

            pObjects->displayData.bUniformColor = true;

            pObjects->Display2();

            visualizer.Run();
        }

        fclose(fpDescriptors);
        fclose(fpDescriptorsMerged);
        // fclose(fpVertices);
        fclose(fpConvexVertices);
        fclose(fpConcaveVertices);
        // fclose(fpVertices);

        RVL_DELETE_ARRAY(NHull.Element);
        delete[] NS;
        delete[] iV;

        if (pMem0 != NULL)
        {
            CreateVNInstances(pMesh, ppVNModel);

            if (visualizationData.bVisualizeVNInstance)
            {
                VNInstance *pVNModel = *ppVNModel;

                uchar SelectionColor[] = {0, 255, 0};

                RandomColors(SelectionColor, visualizationData.clusterColor, pVNModel->nComponents);

                InitDisplay(visualizationData.pVisualizer, pMesh, SelectionColor);

                visualizationData.pVNSceneInstance = pVNModel;

                pVNModel->Display(visualizationData.pVisualizer, pMesh, pSurfels, visualizationData.clusterColor);

                visualizationData.pVisualizer->Run();

                visualizationData.pVisualizer->renderer->RemoveAllViewProps();
            }
        }

        delete[] dS_;
        delete[] bdS_;

        // END Vidovi�
    }                             // if (trainingMethod == 1)
    else if (trainingMethod == 3) // Saving descriptors computed for all canonical orientations.
    {
        FILE *fpDescriptors = fopen("C:\\RVL\\ExpRez\\descriptors.txt", "a");

        int nR = alignment.CTIDescMap.nCTI;

        float *d = new float[nR * sampledUnitSphere.h];
        uchar *bd = new uchar[nR * sampledUnitSphere.h];

        convexHullVertices.Element = new float[3 * pSurfels->vertexArray.n];

        int iCluster, iR, partID, type;
        int *intR;
        RECOG::SceneCluster *pSCluster;
        RECOG::PSGM_::Cluster *pSCluster_;
        float R0[9], R_[9];
        float *d_;
        uchar *bd_;

        for (iCluster = 0; iCluster < SClusters.n; iCluster++)
        {
            pSCluster = SClusters.Element + iCluster;

            if (pSCluster->type == RVLVN_CLUSTER_TYPE_CONVEX || pSCluster->type == RVLVN_CLUSTER_TYPE_CONCAVE)
            {
                pSCluster_ = (RECOG::PSGM_::Cluster *)(pSCluster->vpCluster);

                partID = (pMesh->bLabels ? pSurfels->NodeArray.Element[pSCluster_->iSurfelArray.Element[0]].ObjectID : 0);

                type = (pSCluster->type == RVLVN_CLUSTER_TYPE_CONVEX ? 0 : 1);

                ComputeDescriptors(pSCluster_->iVertexArray, R0, d, bd, true);

                for (iR = 0; iR < nR; iR++)
                {
                    d_ = d + sampledUnitSphere.h * iR;
                    bd_ = bd + sampledUnitSphere.h * iR;

                    intR = alignment.CTIDescMap.R + 9 * iR;

                    RVLMXMUL3X3(R0, intR, R_);

                    SaveDescriptor(fpDescriptors, d_, bd_, R_, sceneID, partID, iCluster, type);
                }
            }
        }

        fclose(fpDescriptors);

        delete[] convexHullVertices.Element;
    }

    // delete[] SClusters.Element;
    // delete[] clusterMap;

    // clusterMap = NULL;
}

void VNClassifier::CreateVNInstances(Mesh *pMesh, VNInstance **ppVNModel)
{
    // RECOG::CreateObjectGraphFromSceneClusters(SClusters, pObjects, pSurfels);

    int iSCluster;

    // for (iSCluster = 0; iSCluster < SClusters.n; iSCluster++)
    //{
    //	pSCluster = SClusters.Element + iSCluster;
    //	pObject = pObjects->objectArray.Element + iSCluster;
    //	if (pSCluster->type == RVLVN_CLUSTER_TYPE_CONVEX || pSCluster->type == RVLVN_CLUSTER_TYPE_CONCAVE)
    //	{
    //		pSCluster_ = (RECOG::PSGM_::Cluster *)(pSCluster->vpCluster);

    //		pObject->d = pSCluster_->d;
    //		pObject->bd = pSCluster_->bd;

    //	}
    //}

    std::vector<int> validObjects;

    for (iSCluster = 0; iSCluster < pObjects->objectArray.n; iSCluster++)
        // validObjects[iSCluster] = iSCluster;
        validObjects.push_back(iSCluster);

    pObjects->SurfelCells(validObjects);

    QList<VNInstance> *pModelVNList = &modelVNList;

    VNInstance VNInstanceTemplate;

    VNInstance *pVNModel;
    RVLMEM_ALLOC_STRUCT(pMem0, VNInstance, pVNModel);
    *pVNModel = VNInstanceTemplate;
    if (ppVNModel)
        *ppVNModel = pVNModel;

    pVNModel->bLabels = pMesh->bLabels;

    int nComponents = pObjects->objectArray.n;

    pVNModel->nComponents = nComponents; // nSCClusters + nSUClusters;

    // RVLMEM_ALLOC_STRUCT_ARRAY(pMem0, bool, nComponents, pVNModel->bConcave);
    // RVLMEM_ALLOC_STRUCT_ARRAY(pMem0, float, 66 * nComponents, pVNModel->d);
    RVLMEM_ALLOC_STRUCT_ARRAY(pMem0, bool, nComponents, pVNModel->bConcave);
    RVLMEM_ALLOC_STRUCT_ARRAY(pMem0, float, nComponents *sampledUnitSphere.h, pVNModel->d);
    RVLMEM_ALLOC_STRUCT_ARRAY(pMem0, SURFEL::Cell, pObjects->surfelCells.n, pVNModel->surfelCells.Element);
    RVLMEM_ALLOC_STRUCT_ARRAY(pMem0, Array<int>, nComponents, pVNModel->componentSurfelCells);
    RVLMEM_ALLOC_STRUCT_ARRAY(pMem0, int, nComponents, pVNModel->label);

    pVNModel->nObjectCellRelations = 0;

    for (iSCluster = 0; iSCluster < pObjects->objectArray.n; iSCluster++)
        pVNModel->nObjectCellRelations += pObjects->objectCells[iSCluster].n;

    RVLMEM_ALLOC_STRUCT_ARRAY(pMem0, int, pVNModel->nObjectCellRelations, pVNModel->componentSurfelCellMem);

    pVNModel->surfelCells.n = pObjects->surfelCells.n;

    for (int i = 0; i < pObjects->surfelCells.n; i++)
    {
        pVNModel->surfelCells.Element[i] = pObjects->surfelCells.Element[i];
    }

    int *pComponentSurfelRelation = pVNModel->componentSurfelCellMem;

    for (iSCluster = 0; iSCluster < pObjects->objectArray.n; iSCluster++)
    {
        pVNModel->componentSurfelCells[iSCluster].Element = pComponentSurfelRelation;

        for (int i = 0; i < pObjects->objectCells[iSCluster].n; i++)
        {
            pVNModel->componentSurfelCells[iSCluster].Element[i] = pObjects->objectCells[iSCluster].Element[i];
        }

        pComponentSurfelRelation += pObjects->objectCells[iSCluster].n;
        pVNModel->componentSurfelCells[iSCluster].n = pObjects->objectCells[iSCluster].n;
    }

    int maxlabel = 0;
    SURFEL::Object *pObject;
    for (int i = 0; i < nComponents; i++)
    {
        pObject = pObjects->objectArray.Element + i;

        memcpy(pVNModel->d + i * sampledUnitSphere.h, pObject->d, sampledUnitSphere.h * sizeof(float));

        pVNModel->bConcave[i] = ((pObject->flags & RVLPCSEGMENT_OBJECT_FLAG_CONCAVE) != 0);

        if (pVNModel->bLabels)
        {
            pVNModel->label[i] = pObject->label;
            if (pVNModel->label[i] > maxlabel)
                maxlabel = pVNModel->label[i];
        }
        else
        {
            pVNModel->label[i] = 0;
        }
    }

    int k;
    pVNModel->supportSize = 0;

    bool *bConsidered = new bool[pVNModel->surfelCells.n];

    memset(bConsidered, 0, pVNModel->surfelCells.n * sizeof(bool));

    for (int i = 0; i < pVNModel->nComponents; i++)
    {
        for (int j = 0; j < pVNModel->componentSurfelCells[i].n; j++)
        {
            k = pVNModel->componentSurfelCells[i].Element[j];

            if (!bConsidered[k])
            {
                bConsidered[k] = true;

                pVNModel->supportSize += pVNModel->surfelCells.Element[k].size;
            }
        }
    }

    delete[] bConsidered;

    int *partMem;
    int j;
    int nTypes = 2;
    pVNModel->part.h = nTypes; // convex and concave
    pVNModel->part.w = maxlabel + 1;
    RVLMEM_ALLOC_STRUCT_ARRAY(pMem0, Array<int>, pVNModel->part.h * pVNModel->part.w, pVNModel->part.Element);
    RVLMEM_ALLOC_STRUCT_ARRAY(pMem0, int, nComponents, partMem);
    int *pPartLabel = partMem;

    for (int ou = 0; ou <= 1; ou++)
    {
        for (int label = 0; label < pVNModel->part.w; label++)
        {
            pVNModel->part.Element[ou * pVNModel->part.w + label].n = 0;
            pVNModel->part.Element[ou * pVNModel->part.w + label].Element = pPartLabel;
            j = ou * pVNModel->part.w + label;

            for (int iComponent = 0; iComponent < nComponents; iComponent++)
            {
                if (pVNModel->bConcave[iComponent] == ou && pVNModel->label[iComponent] == label)
                    pVNModel->part.Element[j].Element[pVNModel->part.Element[j].n++] = iComponent;
            }
            pPartLabel += pVNModel->part.Element[j].n;
        }
    }
}

void VNClassifier::CreateConvexHullsAndBoundingSpheres()
{
    if (modelDataBase == NULL)
        return;

    char *CHModelDataBaseFileName = RVLCreateFileName(modelDataBase, ".dat", -1, ".ch.dat");

    convexHullVertices.w = 3;

    int nVertices, iTmp, iVertex;
    float *P;
    Sphere<float> *pSphere;

    FILE *fpVertices = fopen(CHModelDataBaseFileName, "r");

    delete[] CHModelDataBaseFileName;

    if (fpVertices == NULL)
    {
        printf("Cannot find convex hull vertex file!\n");

        return;
    }

    printf("Creating model convex hulls and bounding spheres...");

    int nModels = pShapeInstanceDetection->nModels;

    RVL_DELETE_ARRAY(modelCH);

    modelCH = new Mesh *[nModels];

    RVL_DELETE_ARRAY(modelBoundingSphere);

    modelBoundingSphere = new Sphere<float>[nModels];

    int iModel;

    for (iModel = 0; iModel < nModels; iModel++)
    {
        modelCH[iModel] = new Mesh;

        modelCH[iModel]->NodeArray.n = 0;
        modelBoundingSphere[iModel].radius = 0.0f;
    }

    maxnModelCHFaces = 0;

    while (!feof(fpVertices))
    {
        fscanf(fpVertices, "%d\t%d\t%d\n", &iModel, &nVertices, &iTmp);

        convexHullVertices.Element = new float[3 * nVertices];
        convexHullVertices.h = nVertices;

        for (iVertex = 0; iVertex < nVertices; iVertex++)
        {
            P = convexHullVertices.Element + 3 * iVertex;

            fscanf(fpVertices, "%f\t%f\t%f\n", P, P + 1, P + 2);

            if (pShapeInstanceDetection->bModelsInMillimeters)
                RVLSCALE3VECTOR(P, 0.001f, P);
        }

        modelCH[iModel]->ConvexHull(convexHullVertices, pMem0, false, false);

        if (modelCH[iModel]->faces.n > maxnModelCHFaces)
            maxnModelCHFaces = modelCH[iModel]->faces.n;

        pSphere = modelBoundingSphere + iModel;

        BoundingSphere<float>(convexHullVertices, pSphere->center, pSphere->radius);

        delete[] convexHullVertices.Element;
    }

    fclose(fpVertices);

    printf("completed.\n");
}

void VNClassifier::CreateSuperSegments(int minClusterSize)
{
    int maxNObjectSegments = 2;

    QList<VN_::SuperSegment> superSegmentList;

    QList<VN_::SuperSegment> *pSuperSegmentList = &superSegmentList;

    RVLQLIST_INIT(pSuperSegmentList);

    Array<int> segments;
    segments.Element = new int[maxNObjectSegments];

    superSegments.n = 0;

    VN_::SuperSegment *pSuperSegment = NULL;

    int i, iSegment, nObjectSegments, iFirstSegment;
    uchar type;
    VN_::SuperSegment *pNewSuperSegment;
    SURFEL::Object *pSegment;

    do
    {
        if (pSuperSegment)
        {
            iFirstSegment = (pSuperSegment->segments.n < maxNObjectSegments ? pSuperSegment->segments.Element[pSuperSegment->segments.n - 1] + 1 : pObjects->objectArray.n);

            type = (pObjects->objectArray.Element[pSuperSegment->segments.Element[0]].flags & RVLPCSEGMENT_OBJECT_FLAG_CONCAVE);
        }
        else
            iFirstSegment = 0;

        for (iSegment = iFirstSegment; iSegment < pObjects->objectArray.n; iSegment++)
        {
            pSegment = pObjects->objectArray.Element + iSegment;

            if (pSegment->flags & RVLPCSEGMENT_OBJECT_FLAG_GND)
                continue;

            if (!bConcaveSuperSegments)
                if (pSegment->flags & RVLPCSEGMENT_OBJECT_FLAG_CONCAVE)
                    continue;

            if (pSegment->size < minClusterSize)
                continue;

            if (pSegment->iVertexArray.n == 0)
                continue;

            if (pSuperSegment)
            {
                if ((pSegment->flags & RVLPCSEGMENT_OBJECT_FLAG_CONCAVE) != type)
                    continue;

                segments.n = pSuperSegment->segments.n + 1;

                for (i = 0; i < pSuperSegment->segments.n; i++)
                    segments.Element[i] = pSuperSegment->segments.Element[i];

                segments.Element[i] = iSegment;
            }
            else
            {
                segments.n = 1;

                segments.Element[0] = iSegment;

                type = (pSegment->flags & RVLPCSEGMENT_OBJECT_FLAG_CONCAVE);
            }

            pNewSuperSegment = CreateSupersegment(segments, (type == RVLPCSEGMENT_OBJECT_FLAG_CONCAVE ? -1.0f : 1.0f));

            if (pNewSuperSegment)
            {
                // if (superSegments.n == 20)
                //	int debug = 0;

                pNewSuperSegment->ID = superSegments.n;

                RVLQLIST_ADD_ENTRY(pSuperSegmentList, pNewSuperSegment);

                superSegments.n++;
            }
        }

        pSuperSegment = (pSuperSegment ? pSuperSegment->pNext : superSegmentList.pFirst);
    } while (pSuperSegment);

    delete[] segments.Element;

    RVL_DELETE_ARRAY(superSegments.Element);

    if (superSegments.n > 0)
    {
        superSegments.Element = new VN_::SuperSegment *[superSegments.n];

        QList<VN_::SuperSegment> *pSuperSegmentList = &superSegmentList;

        Array<VN_::SuperSegment *> *pSuperSegmentArray = &superSegments;

        QLIST::CreatePtrArray<VN_::SuperSegment>(pSuperSegmentList, pSuperSegmentArray);
    }
}

void VNClassifier::DetectGroundPlane(Mesh *pMesh)
{
    Array<int> PtArray;

    PtArray.Element = new int[pMesh->NodeArray.n];

    bGnd = false;

    int maxSize = 0;

    int iSegment;
    SURFEL::Object *pObject, *pGroundPlaneSeed;
    float N[3];
    float d;

    for (iSegment = 0; iSegment < pObjects->objectArray.n; iSegment++)
    {
        pObject = pObjects->objectArray.Element + iSegment;

        if (pShapeInstanceDetection->IsFlat(pObject->iSurfelArray, N, d, PtArray))
        {
            bGnd = true;

            if (pObject->size > maxSize)
            {
                pGroundPlaneSeed = pObject;

                RVLCOPY3VECTOR(N, NGnd);

                dGnd = d;

                maxSize = pObject->size;
            }
        }
    }

    if (bGnd)
    {
        pGroundPlaneSeed->flags |= RVLPCSEGMENT_OBJECT_FLAG_GND;

        int i;
        float *P;
        float eGnd;

        for (iSegment = 0; iSegment < pObjects->objectArray.n; iSegment++)
        {
            pObject = pObjects->objectArray.Element + iSegment;

            if (pObject == pGroundPlaneSeed)
                continue;

            // FILE *fpDebug = NULL;

            // if (iSegment == 1)
            //	fpDebug = fopen("C:\\RVL\\Debug\\P.txt", "w");

            for (i = 0; i < pObject->iVertexArray.n; i++)
            {
                P = pSurfels->vertexArray.Element[pObject->iVertexArray.Element[i]]->P;

                // if (fpDebug)
                //	fprintf(fpDebug, "%f\t%f\t%f\n", P[0], P[1], P[2]);

                eGnd = RVLDOTPRODUCT3(NGnd, P) - dGnd;

                if (RVLABS(eGnd) > pShapeInstanceDetection->groundPlaneTolerance)
                    break;
            }

            // if (fpDebug)
            //	fclose(fpDebug);

            if (i < pObject->iVertexArray.n)
                continue;

            pObject->flags |= RVLPCSEGMENT_OBJECT_FLAG_GND;
        }
    }

    delete[] PtArray.Element;
}

void VNClassifier::CreateSurfelMask(
    Mesh *pMesh,
    RECOG::VN_::SurfelMask &surfelMask)
{
    int iPt;
    int minSurfelID, maxSurfelID;

    minSurfelID = maxSurfelID = pSurfels->surfelMap[0];

    for (iPt = 0; iPt < pMesh->NodeArray.n; iPt++)
    {
        if (pSurfels->surfelMap[iPt] < minSurfelID)
            minSurfelID = pSurfels->surfelMap[iPt];
        else if (pSurfels->surfelMap[iPt] > maxSurfelID)
            maxSurfelID = pSurfels->surfelMap[iPt];
    }

    if (maxSurfelID < pSurfels->NodeArray.n - 1)
        maxSurfelID = pSurfels->NodeArray.n - 1;

    surfelMask.nSurfelIDs = maxSurfelID - minSurfelID + 1;

    surfelMask.mem = new bool[surfelMask.nSurfelIDs];
    surfelMask.mask = surfelMask.mem - minSurfelID;

    memset(surfelMask.mem, 0, surfelMask.nSurfelIDs * sizeof(bool));
}

void VNClassifier::CreateForegroundObjectSurfelMask(RECOG::VN_::SurfelMask surfelMask)
{
    // RVL_DELETE_ARRAY(pShapeInstanceDetection->imageMask);

    // pShapeInstanceDetection->imageMask = new bool[pMesh->NodeArray.n];

    // for (iPt = 0; iPt < pMesh->NodeArray.n; iPt++)
    //	pShapeInstanceDetection->imageMask[iPt] = pMesh->NodeArray.Element[iPt].bValid;

    // Array<int> PtArray;

    // PtArray.Element = new int[pMesh->NodeArray.n];

    int i, iSegment;
    SURFEL::Object *pObject;

    for (iSegment = 0; iSegment < pObjects->objectArray.n; iSegment++)
    {
        pObject = pObjects->objectArray.Element + iSegment;

        // if (pObject->flags & RVLPCSEGMENT_OBJECT_FLAG_GND)
        //	continue;

        for (i = 0; i < pObject->iSurfelArray.n; i++)
            surfelMask.mask[pObject->iSurfelArray.Element[i]] = true;
    }

    // for (iSegment = 0; iSegment < pObjects->objectArray.n; iSegment++)
    //{
    //	pObject = pObjects->objectArray.Element + iSegment;

    //	if (pObject->flags & RVLPCSEGMENT_OBJECT_FLAG_GND)
    //	{
    //		for (i = 0; i < pObject->iSurfelArray.n; i++)
    //			surfelMask.mask[pObject->iSurfelArray.Element[i]] = false;
    //	}

    //	//if (pObject->flags & RVLPCSEGMENT_OBJECT_FLAG_GND)
    //	//{
    //	//	pSurfels->GetPoints(pObject->iSurfelArray, PtArray);

    //	//	for (i = 0; i < PtArray.n; i++)
    //	//		pShapeInstanceDetection->imageMask[PtArray.Element[i]] = false;
    //	//}
    //}

    // delete[] PtArray.Element;
}

void VNClassifier::CreateSurfelMask(
    Array<int> segments,
    RECOG::VN_::SurfelMask surfelMask,
    bool b)
{
    int i, j, iSegment;
    SURFEL::Object *pSegment;

    for (i = 0; i < segments.n; i++)
    {
        iSegment = segments.Element[i];

        pSegment = pObjects->objectArray.Element + iSegment;

        for (j = 0; j < pSegment->iSurfelArray.n; j++)
            surfelMask.mask[pSegment->iSurfelArray.Element[j]] = b;
    }
}

void VNClassifier::DisplaySurfelMask(
    VN_::SurfelMask surfelMask,
    Rect<int> *pBBox)
{
    cv::Mat displayImage(camera.h, camera.w, CV_8UC3);

    unsigned char *pixTgt = displayImage.data;

    int nPix = camera.h * camera.w;

    int iPt;

    for (iPt = 0; iPt < nPix; iPt++, pixTgt += 3)
        if (surfelMask.mask[pSurfels->surfelMap[iPt]])
            RVLSET3VECTOR(pixTgt, 255, 255, 255)
        else
            RVLSET3VECTOR(pixTgt, 0, 0, 0);

    if (pBBox)
    {
        cv::line(displayImage, cv::Point(pBBox->minx, pBBox->miny), cv::Point(pBBox->minx, pBBox->maxy), cv::Scalar(0, 0, 255), 2);
        cv::line(displayImage, cv::Point(pBBox->minx, pBBox->maxy), cv::Point(pBBox->maxx, pBBox->maxy), cv::Scalar(0, 0, 255), 2);
        cv::line(displayImage, cv::Point(pBBox->maxx, pBBox->maxy), cv::Point(pBBox->maxx, pBBox->miny), cv::Scalar(0, 0, 255), 2);
        cv::line(displayImage, cv::Point(pBBox->maxx, pBBox->miny), cv::Point(pBBox->minx, pBBox->miny), cv::Scalar(0, 0, 255), 2);
    }

    cv::imshow("mask", displayImage);

    cv::waitKey();
}

void VNClassifier::GreedyInterpretation2()
{
    // Sort all Level3 hypotheses.

    Array<VN_::HypothesisSortPtr> level3Hypotheses;

    level3Hypotheses.Element = new VN_::HypothesisSortPtr[superSegments.n * nHypothesesLevel3];

    level3Hypotheses.n = 0;

    int i, iSuperSegment, nLevel3Hypotheses;

    for (iSuperSegment = 0; iSuperSegment < superSegments.n; iSuperSegment++)
    {
        nLevel3Hypotheses = RVLMIN(nHypothesesLevel3, sortedSuperSegmentHypotheses[iSuperSegment].n);

        for (i = 0; i < nLevel3Hypotheses; i++)
            level3Hypotheses.Element[level3Hypotheses.n++] = sortedSuperSegmentHypotheses[iSuperSegment].Element[i];
    }

    BubbleSort<VN_::HypothesisSortPtr>(level3Hypotheses);

    // Starting from the hypothesis with the lowest cost, copy all hypotheses, which are not in collision with a lower cost hypothesis to interpretation2.

    bool *bOccupied = new bool[pObjects->objectArray.n];

    memset(bOccupied, 0, pObjects->objectArray.n * sizeof(bool));

    interpretation2.clear();

    int j, segmentSize, size, iSegment, original;
    VN_::Hypothesis2 *pHypothesis;

    for (i = 0; i < level3Hypotheses.n; i++)
    {
        pHypothesis = level3Hypotheses.Element[i].ptr;

        if (bUseColor)
        {
            if (pHypothesis->matchedPtsPercentage < greedyInterpretationMatchedPercentageThr || pHypothesis->score > 0.0f || pHypothesis->CHMatchingMetric > greedyInterpretationColorThr)
                continue;
        }
        else if (pHypothesis->matchedPtsPercentage < greedyInterpretationMatchedPercentageThr || pHypothesis->score > 0.0f)
            continue;

        size = original = 0;

        for (j = 0; j < pHypothesis->segments.n; j++)
        {
            iSegment = pHypothesis->segments.Element[j];

            segmentSize = pObjects->objectArray.Element[iSegment].size;

            size += segmentSize;

            if (!bOccupied[iSegment])
                original += segmentSize;
        }

        if (size > 0)
        {
            if (100 * original / size >= collisionPercThr)
            {
                interpretation2.push_back(pHypothesis);

                for (j = 0; j < pHypothesis->segments.n; j++)
                {
                    iSegment = pHypothesis->segments.Element[j];

                    bOccupied[iSegment] = true;
                }
            }
        }
        // else
        //	int debug = 0;
    }

    // Free memory.

    delete[] level3Hypotheses.Element;
    delete[] bOccupied;
}

bool VNClassifier::HypothesisEvaluationCH(
    RECOG::VN_::Hypothesis2 *pHypothesis,
    float &eavg,
    float &emax)
{
    // Parameters.

    float tolerance = 0.04f;

    //

    float *E = D;
    float *E_ = E + maxnModelCHFaces;
    float *dMS = E_ + maxnModelCHFaces;

    float PBBoxS[3];

    float *PBBoxM = modelBoundingSphere[pHypothesis->iModel].center;

    float *RMS = pHypothesis->R;
    float *tMS = pHypothesis->P;

    RVLTRANSF3(PBBoxM, RMS, tMS, PBBoxS);

    float rBBox2 = modelBoundingSphere[pHypothesis->iModel].radius + tolerance;

    iVertexArray.n = 0;

    rBBox2 *= rBBox2;

    int i, iVertex;
    float *PS;
    float dP[3];

    for (i = 0; i < iObjectVertexArray.n; i++)
    {
        iVertex = iObjectVertexArray.Element[i];

        PS = pSurfels->vertexArray.Element[iVertex]->P;

        RVLDIF3VECTORS(PS, PBBoxS, dP);

        if (RVLDOTPRODUCT3(dP, dP) <= rBBox2)
        {
            bVertexInArray[iVertex] = true;

            iVertexArray.Element[iVertexArray.n++] = iVertex;
        }
    }

    Array<MESH::Face *> faces = modelCH[pHypothesis->iModel]->faces;

    float *NS = NBuff;

    int j;
    float *NM;

    for (j = 0; j < faces.n; j++, NS += 3)
    {
        NM = faces.Element[j]->N;

        RVLMULMX3X3VECT(RMS, NM, NS);

        dMS[j] = RVLDOTPRODUCT3(NS, tMS) + faces.Element[j]->d;
    }

    // iVertexArray2.n = 0;

    bool bSegmentsInCH = false;

    int iSegment;
    SURFEL::Object *pSegment;
    MESH::Face *pFace;
    float d, dmax, e;
    float *N;

    for (iSegment = 0; iSegment < pObjects->objectArray.n; iSegment++)
    {
        pSegment = pObjects->objectArray.Element + iSegment;

        for (i = 0; i < pSegment->iVertexArray.n; i++)
        {
            if (!bVertexInArray[pSegment->iVertexArray.Element[i]])
                break;
        }

        if (i < pSegment->iVertexArray.n) // pSegment is not completely contained in the hypothesis bounding box.
            continue;

        for (i = 0; i < pSegment->iVertexArray.n; i++)
        {
            iVertex = pSegment->iVertexArray.Element[i];

            PS = pSurfels->vertexArray.Element[iVertex]->P;

            NS = NBuff;

            if (i == 0)
            {
                for (j = 0; j < faces.n; j++, NS += 3)
                {
                    d = RVLDOTPRODUCT3(NS, PS);

                    e = d - dMS[j];

                    if (e > tolerance)
                        break;

                    E_[j] = e;
                }

                if (j < faces.n) // pSegment is not completely contained in the hypothesis convex hull.
                    break;
            }
            else
            {
                for (j = 0; j < faces.n; j++, NS += 3)
                {
                    d = RVLDOTPRODUCT3(NS, PS);

                    e = d - dMS[j];

                    if (e > tolerance)
                        break;

                    if (e > E_[j])
                        E_[j] = e;
                }

                if (j < faces.n) // pSegment is not completely contained in the hypothesis convex hull.
                    break;
            }
        }

        if (i < pSegment->iVertexArray.n) // pSegment is not completely contained in the hypothesis convex hull.
            continue;

        if (bSegmentsInCH)
        {
            for (j = 0; j < faces.n; j++)
                if (E_[j] > E[j])
                    E[j] = E_[j];
        }
        else
        {
            for (j = 0; j < faces.n; j++)
                E[j] = E_[j];

            bSegmentsInCH = true;
        }
    } // for every segment

    for (i = 0; i < iVertexArray.n; i++)
        bVertexInArray[iVertexArray.Element[i]] = false;

    if (!bSegmentsInCH)
        return false;

    int nVisibleFaces = 0;

    eavg = emax = 0.0f;

    for (j = 0; j < faces.n; j++)
    {
        if (dMS[j] >= 0.0f)
            continue;

        nVisibleFaces++;

        e = RVLABS(E[j]);

        eavg += e;

        if (e > emax)
            emax = e;
    }

    eavg /= (float)(nVisibleFaces);

    // for (i = 0; i < iVertexArray2.n; i++)
    //	bVertexInArray2[iVertexArray2.Element[i]] = false;

    return true;
}

void VNClassifier::AssignSegmentsToHypothesis(
    RECOG::VN_::Hypothesis2 *pHypothesis,
    float tolerance)
{
    // iVertexArray <- vertices contained in the hypothesis CH.
    // bVertexInArray <- indicates whether a particular vertex is contained in the hypothesis CH.

    float PBBoxS[3];

    float *PBBoxM = modelBoundingSphere[pHypothesis->iModel].center;

    float *RMS = pHypothesis->R;
    float *tMS = pHypothesis->P;

    RVLTRANSF3(PBBoxM, RMS, tMS, PBBoxS);

    float rBBox2 = modelBoundingSphere[pHypothesis->iModel].radius + tolerance;

    iVertexArray.n = 0;

    rBBox2 *= rBBox2;

    int i, iVertex;
    float *PS;
    float dP[3];

    for (i = 0; i < iObjectVertexArray.n; i++)
    {
        iVertex = iObjectVertexArray.Element[i];

        PS = pSurfels->vertexArray.Element[iVertex]->P;

        RVLDIF3VECTORS(PS, PBBoxS, dP);

        if (RVLDOTPRODUCT3(dP, dP) <= rBBox2)
        {
            bVertexInArray[iVertex] = true;

            iVertexArray.Element[iVertexArray.n++] = iVertex;
        }
    }

    // Transform the hypothesis model CH to the scene.

    Array<MESH::Face *> faces = modelCH[pHypothesis->iModel]->faces;

    float *NS = NBuff;
    float *dMS = D;

    int j;
    float *NM;

    for (j = 0; j < faces.n; j++, NS += 3)
    {
        NM = faces.Element[j]->N;

        RVLMULMX3X3VECT(RMS, NM, NS);

        dMS[j] = RVLDOTPRODUCT3(NS, tMS) + faces.Element[j]->d;
    }

    // pHypothesis->segments <- indices of the segments contained inside the hypothesis CH.

    // iVertexArray2.n = 0;

    int iSegment;
    SURFEL::Object *pSegment;
    MESH::Face *pFace;
    float d, dmax, e;
    float *N;

    for (iSegment = 0; iSegment < pObjects->objectArray.n; iSegment++)
    {
        pSegment = pObjects->objectArray.Element + iSegment;

        if (pSegment->iVertexArray.n == 0)
            continue;

        for (i = 0; i < pSegment->iVertexArray.n; i++)
        {
            if (!bVertexInArray[pSegment->iVertexArray.Element[i]])
                break;
        }

        if (i < pSegment->iVertexArray.n) // pSegment is not completely contained in the hypothesis bounding box.
            continue;

        for (i = 0; i < pSegment->iVertexArray.n; i++)
        {
            iVertex = pSegment->iVertexArray.Element[i];

            PS = pSurfels->vertexArray.Element[iVertex]->P;

            NS = NBuff;

            if (i == 0)
            {
                for (j = 0; j < faces.n; j++, NS += 3)
                {
                    d = RVLDOTPRODUCT3(NS, PS);

                    e = d - dMS[j];

                    if (e > tolerance)
                        break;
                }

                if (j < faces.n) // pSegment is not completely contained in the hypothesis convex hull.
                    break;
            }
            else
            {
                for (j = 0; j < faces.n; j++, NS += 3)
                {
                    d = RVLDOTPRODUCT3(NS, PS);

                    e = d - dMS[j];

                    if (e > tolerance)
                        break;
                }

                if (j < faces.n) // pSegment is not completely contained in the hypothesis convex hull.
                    break;
            }
        }

        if (i < pSegment->iVertexArray.n) // pSegment is not completely contained in the hypothesis convex hull.
            continue;

        pHypothesis->segments.Element[pHypothesis->segments.n++] = iSegment;
    } // for every segment

    for (i = 0; i < iVertexArray.n; i++)
        bVertexInArray[iVertexArray.Element[i]] = false;
}

void VNClassifier::SaveVNInstances(
    QList<VNInstance> *pModelVNList,
    void *vpClassifier)
{
    if (ModelVNInstanceListFileName == NULL)
        return;
    FILE *fpModelVNList;
    if (bGenerateSceneVNInstances)
        fpModelVNList = fopen(SceneVNInstanceListFileName, "w");
    if (bGenerateModelVNInstances)
        fpModelVNList = fopen(ModelVNInstanceListFileName, "w");

    VNClassifier *pVNClassifier = (VNClassifier *)vpClassifier;
    VNInstance *pVNModelInstance = pModelVNList->pFirst;

    while (pVNModelInstance)
    {
        pVNModelInstance->SaveVNInstance(pVNClassifier, fpModelVNList);
        pVNModelInstance = pVNModelInstance->pNext;
    }

    fprintf(fpModelVNList, "end");

    fclose(fpModelVNList);
}

void VNClassifier::LoadVNInstances(void *vpClassifier, char *VNInstanceListFileName, QList<VNInstance> *pVNList, Array<VNInstance *> *pVNArray)
{
    VNClassifier *pVNClassifier = (VNClassifier *)vpClassifier;

    RVLQLIST_INIT(pVNList);
    FILE *fp;

    fp = fopen(VNInstanceListFileName, "r");

    if (fp == NULL)
        return;

    int nModels = 0;

    VNInstance *pVNModel;

    while (true)
    {
        char line[500];
        int i;
        fgets(line, 500, fp);
        if (strcmp(line, "end") == 0)
            break;

        VNInstance VNInstanceTemplate;

        RVLMEM_ALLOC_STRUCT(pMem0, VNInstance, pVNModel);
        *pVNModel = VNInstanceTemplate;
        RVLQLIST_ADD_ENTRY(pVNList, pVNModel);

        pVNModel->ID = nModels;

        pVNModel->LoadVNInstance(pVNClassifier, fp, pMem0);

        pVNModel->ProjectToLatentSubspace(pVNClassifier);

        nModels++;
    }

    fclose(fp);

    RVL_DELETE_ARRAY(pVNArray->Element);
    pVNArray->Element = new VNInstance *[nModels];
    QLIST::CreatePtrArray<VNInstance>(pVNList, pVNArray);

    RVL_DELETE_ARRAY(firstComponentAbsIdx.Element);

    firstComponentAbsIdx.Element = new int[nModels];

    int iMCompAbs = 0;

    int iModel;

    for (iModel = 0; iModel < modelVNArray.n; iModel++)
    {
        pVNModel = modelVNArray.Element[iModel];

        firstComponentAbsIdx.Element[iModel] = iMCompAbs;

        iMCompAbs += pVNModel->nComponents;
    }

    RVL_DELETE_ARRAY(modelSet.Element);

    modelSet.Element = new int[modelVNArray.n];

    if (componentFileName)
    {
        FILE *fpComp = fopen(componentFileName, "r");

        modelSet.n = 0;

        bool *bAlreadyInSubset = new bool[modelVNArray.n];

        memset(bAlreadyInSubset, 0, modelVNArray.n * sizeof(bool));

        int readCount, iComp;

        while (!feof(fpComp))
        {
            // readCount = fscanf(fpComp, "%d\t%d\n", &iModel, &iComp);
            readCount = fscanf(fpComp, "%d\n", &iModel);

            if (readCount == 1)
            {
                if (!bAlreadyInSubset[iModel])
                {
                    bAlreadyInSubset[iModel] = true;

                    modelSet.Element[modelSet.n++] = iModel;
                }
            }
            // else
            //	break;
        }

        fclose(fpComp);

        delete[] bAlreadyInSubset;
    }
    else
    {
        for (iModel = 0; iModel < modelVNArray.n; iModel++)
            modelSet.Element[iModel] = iModel;

        modelSet.n = modelVNArray.n;
    }

    int i;
    VNInstance *pModel;

    for (i = 0; i < modelSet.n; i++)
    {
        pModel = modelVNArray.Element[modelSet.Element[i]];

        pModel->ComputeTREDMx(this);
    }
}

void VNClassifier::LoadPCs()
{
    FileSequenceLoader modelsLoader;

    modelsLoader.Init(modelSequenceFileName);

    printf("Loading point clouds...\n");

    int iModel = 0;

    char modelFilePath[200];
    char modelFileName[200];
    VNInstance *pModel;

    while (modelsLoader.GetNext(modelFilePath, modelFileName))
    {
        pModel = modelVNArray.Element[iModel];

        pModel->LoadPC(modelFilePath);

        iModel++;

        printf("%d%%\r", 100 * iModel / modelVNArray.n);
    }

    printf("100%%\n");
}

float VNClassifier::ComputeDescriptor(
    Mesh *pMesh,
    float *RIn,
    float *tIn,
    float *&dS,
    bool *&bdS,
    Box<float> &SBoundingBox,
    int iModel,
    float score)
{
    // Detect surfels.

    pSurfels->Init(pMesh);

    pSurfelDetector->Init(pMesh, pSurfels, pMem);

    printf("Segmentation to surfels...");

    pSurfelDetector->Segment(pMesh, pSurfels);

    printf("completed.\n");

    int nSurfels = pSurfels->NodeArray.n;

    printf("No. of surfels = %d\n", nSurfels);

    pSurfels->DetectVertices(pMesh);

    // Create scene object.

    float *R = sceneObject.R;

    if (RIn)
    {
        RVLCOPYMX3X3(RIn, R);
    }
    else
    {
        RVLUNITMX3(R);
    }

    float *t = sceneObject.t;

    if (tIn)
    {
        RVLCOPY3VECTOR(tIn, t);
    }
    else
    {
        RVLNULL3VECTOR(t);
    }

    sceneObject.vertexArray = new float[3 * pSurfels->vertexArray.n];

    float *P = sceneObject.vertexArray;

    int iVertex;
    SURFEL::Vertex *pVertex;

    for (iVertex = 0; iVertex < pSurfels->vertexArray.n; iVertex++, P += 3)
    {
        pVertex = pSurfels->vertexArray.Element[iVertex];

        RVLTRANSF3(pVertex->P, R, t, P);
    }

    sceneObject.NArray = new float[3 * pSurfels->NodeArray.n];

    float *N = sceneObject.NArray;

    int iSurfel;
    Surfel *pSurfel;

    for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++, N += 3)
    {
        pSurfel = pSurfels->NodeArray.Element + iSurfel;

        RVLMULMX3X3VECT(R, pSurfel->N, N);
    }

    int nSamplePts = 300;

    pSurfels->SampleMeshDistanceFunction(pMesh, R, t, nSamplePts, voxelSize, sampleVoxelDistance, sceneObject.sampleArray);

    // Cluster surfels into convex and concave surfaces.

    Array<RECOG::SceneCluster> SClusters;

    bool bConcavity, bTorus;

    VN *pModel = (iModel >= 0 ? models[iModel] : NULL);

    if (trainingMethod == 0)
        pModel->ClusterTypes(bConcavity, bTorus);
    else if (trainingMethod == 1)
    {
        bConcavity = true;
        bTorus = false;
    }

    int nSCClusters, nSUClusters;

    PSGM_::ConvexAndConcaveClusters(pMesh, pSurfels, pSurfelDetector, &convexClustering, &concaveClustering,
                                    SClusters, nSCClusters, nSUClusters, 0, NULL, maxnSCClusters, maxnSUClusters, bConcavity, (bTorus ? maxnSTClusters : 0),
                                    visualizationData.bVisualizeConvexClusters, visualizationData.bVisualizeConcaveClusters, visualizationData.bVisualizeSurfels);

    // fpDebug = fopen("debug.txt", "w");

    // fprintf(fpDebug, "nFeatures=%d\n", pModel->featureArray.n);

    // for (int i = 0; i < pModel->featureArray.n; i++)
    //	fprintf(fpDebug, "%f\t%f\t%f\n", pModel->featureArray.Element[i].N[0], pModel->featureArray.Element[i].N[1], pModel->featureArray.Element[i].N[2]);

    // for (int i = pModel->featureArray.n; i < pModel->NodeArray.n; i++)
    //	fprintf(fpDebug, "O%d\n", pModel->NodeArray.Element[i].operation);

    // RECOG::VN_::Edge *pEdge = pModel->EdgeList.pFirst;

    // while (pEdge)
    //{
    //	fprintf(fpDebug, "E%d-%d\n", pEdge->data.a, pEdge->data.b);

    //	pEdge = pEdge->pNext;
    //}

    // fprintf(fpDebug, "#conv.clusters %d, #concave clusters: %d\n", convexClustering.clusters.n, concaveClustering.clusters.n);

    // RECOG::PSGM_::Cluster *pSCluster_;

    // for (int i = 0; i < SClusters.n; i++)
    //{
    //	RECOG::SceneCluster *pSCluster = SClusters.Element + i;

    //	if (pSCluster->type == RVLVN_CLUSTER_TYPE_CONVEX || pSCluster->type == RVLVN_CLUSTER_TYPE_CONCAVE)
    //	{
    //		pSCluster_ = (RECOG::PSGM_::Cluster *)(pSCluster->vpCluster);

    //		fprintf(fpDebug, "%d\n", pSCluster_->size);
    //	}
    //}

    // Cluster surfels into convex surfaces.

    // convexClustering.pMesh = pMesh;

    // convexClustering.Clusters();

    // Cluster surfels into concave surfaces.

    // concaveClustering.pMesh = pMesh;

    // concaveClustering.Clusters();

    // Selection color for visualization.

    // uchar SelectionColor[] = {0, 255, 0};

    // Visualization

    // Visualizer visualizer;

    // visualizer.Create();

    // pSurfels->NodeColors(SelectionColor);

    // Visualization of convex clusters.

    // RVL_DELETE_ARRAY(convexClustering.clusterColor);

    // RandomColors(SelectionColor, convexClustering.clusterColor, convexClustering.clusters.n);

    // convexClustering.InitDisplay(&visualizer, pMesh, SelectionColor);

    // convexClustering.Display();

    // visualizer.Run();

    // pSurfels->DisplayData.bCallbackFunctionsDefined = false;

    // Visualization of concave clusters.

    // RVL_DELETE_ARRAY(concaveClustering.clusterColor);

    // RandomColors(SelectionColor, concaveClustering.clusterColor, concaveClustering.clusters.n);

    // concaveClustering.InitDisplay(&visualizer, pMesh, SelectionColor);

    // concaveClustering.Display();

    // visualizer.Run();

    // pSurfels->DisplayData.bCallbackFunctionsDefined = false;

    // Surfel visualization.

    // surfels.NodeColors(SelectionColor);

    // surfels.InitDisplay(&visualizer, &mesh, &surfelDetector);

    // surfels.Display(&visualizer, &mesh);

    // visualizer.Run();

    // pSurfels->DisplayData.bCallbackFunctionsDefined = false;

    if (trainingMethod == 0)
    {
        printf("Matching VN model to scene...");

        P = sceneObject.vertexArray;

        InitBoundingBox<float>(&SBoundingBox, P);

        P += 3;

        for (iVertex = 1; iVertex < pSurfels->vertexArray.n; iVertex++, P += 3)
            UpdateBoundingBox<float>(&SBoundingBox, P);

        dS = new float[pModel->featureArray.n];

        bdS = new bool[pModel->featureArray.n];

        // pModel->fpDebug = fpDebug;

        pModel->Match4(pMesh, sceneObject, this, SClusters, SBoundingBox, dS, bdS, bTrainingStructure);
        score = pModel->score;

        // fclose(fpDebug);

        delete[] SClusters.Element;
        delete[] sceneObject.vertexArray;
        delete[] sceneObject.NArray;

        printf("completed.\n");
    }
    else if (trainingMethod == 1)
    {
        // Vidovi�
        FILE *fpDescriptors = fopen("C:\\RVL\\ExpRez\\descriptors.txt", "a");
        FILE *fpVertices = fopen("C:\\RVL\\ExpRez\\vertices.txt", "a");
        int iMetaModel = 0;
        // int iModel = 0;
        // VN *pModel_ = models[iModel];

        RECOG::SceneCluster *pSCluster;
        RECOG::PSGM_::Cluster *pSCluster_;

        float R[9];
        RVLUNITMX3(R);

        // NHull is not used
        Array<SURFEL::NormalHullElement> NHull; // not used
        NHull.Element = NULL;
        NHull.n = 0;

        // float *dS_ = new float[pModel_->featureArray.n]; //change 66 to variable
        // uchar *bdS_ = new uchar[pModel_->featureArray.n]; //change 66 to variable
        float *dS_ = new float[sampledUnitSphere.h];
        uchar *bdS_ = new uchar[sampledUnitSphere.h];

        // 3.10.2018. - added because ComputeDescriptor function was changed
        float *NS = new float[sampledUnitSphere.h * sampledUnitSphere.w];
        int *iV = new int[sampledUnitSphere.h];

        for (int i = 0; i < SClusters.n; i++)
        {
            pSCluster = SClusters.Element + i;

            if (pSCluster->type == RVLVN_CLUSTER_TYPE_CONVEX || pSCluster->type == RVLVN_CLUSTER_TYPE_CONCAVE)
            {
                pSCluster_ = (RECOG::PSGM_::Cluster *)(pSCluster->vpCluster);

                if (pSCluster->type == RVLVN_CLUSTER_TYPE_CONVEX)
                {
                    // ComputeDescriptor(pSCluster_->iVertexArray, NHull, R, dS_, bdS_, 1.0F, true, false);
                    // SaveDescriptor(fpDescriptors, dS_, bdS_, 0, iMetaModel);
                    ComputeDescriptor(pSCluster_->iVertexArray, NHull, R, dS_, bdS_, NS, iV, 1.0F, true, false);
                    // SaveDescriptor(fpDescriptors, dS_, (bool*)bdS_, 0, iMetaModel);
                    SaveDescriptor(fpDescriptors, dS_, bdS_, R, i, 0);
                }
                else
                {
                    // ComputeDescriptor(pSCluster_->iVertexArray, NHull, R, dS_, bdS_, -1.0F, true, false);
                    // SaveDescriptor(fpDescriptors, dS_, bdS_, 1, iMetaModel);
                    ComputeDescriptor(pSCluster_->iVertexArray, NHull, R, dS_, bdS_, NS, iV, -1.0F, true, false);
                    // SaveDescriptor(fpDescriptors, dS_, (bool*)bdS_, 1, iMetaModel);
                    SaveDescriptor(fpDescriptors, dS_, bdS_, R, i, 1);
                }
            }

            // SaveVertices(fpVertices, pSCluster_);
        }

        fclose(fpDescriptors);
        fclose(fpVertices);

        delete[] dS_;
        delete[] bdS_;
        RVL_DELETE_ARRAY(NHull.Element);
        delete[] NS;
        delete[] iV;

        // END Vidovi�
    }
    return score;
}

void VNClassifier::ComputeDescriptors(
    Array<int> iSurfelArray,
    Array<int> iVertexArray,
    float *d,
    uchar *bd,
    float *dU,
    uchar *bdU)
{
    // NHull <- convex hull of all surfel normals from iSurfelArray (Tried, but it was not successful.)

    Array<SURFEL::NormalHullElement> NHull;

    NHull.Element = new SURFEL::NormalHullElement[iSurfelArray.n];
    NHull.n = 0;

    int i;
    int iSurfel;
    Surfel *pSurfel;

    for (i = 0; i < iSurfelArray.n; i++)
    {
        iSurfel = iSurfelArray.Element[i];

        pSurfel = pSurfels->NodeArray.Element + iSurfel;

        pSurfels->UpdateNormalHull(NHull, pSurfel->N);
    }

    // Only for debugging purpose!

    // float maxDistFromNHull = -2.0;

    // for (i = 0; i < iSurfelArray.n; i++)
    //{
    //	iSurfel = iSurfelArray.Element[i];

    //	pSurfel = pSurfels->NodeArray.Element + iSurfel;

    //	float distFromNHull = pSurfels->DistanceFromNormalHull(NHull, pSurfel->N);

    //	if (distFromNHull > maxDistFromNHull)
    //		maxDistFromNHull = distFromNHull;
    //}

    float *NS = new float[sampledUnitSphere.h * sampledUnitSphere.w];

    int *iV = new int[sampledUnitSphere.h];

    // int debug = 0;

    ///

    // Compute a CTI descriptor for every rotation in SO3Samples.

    int iR;
    float *d_;
    uchar *bd_;
    float *R;

    for (iR = 0; iR < SO3Samples.c; iR++)
    {
        R = SO3Samples.Element + 9 * iR;

        d_ = d + sampledUnitSphere.h * iR;

        bd_ = bd + sampledUnitSphere.h * iR;

        ComputeDescriptor(iVertexArray, NHull, R, d_, bd_, NS, iV);

        if (dU)
        {
            d_ = dU + sampledUnitSphere.h * iR;

            bd_ = bdU + sampledUnitSphere.h * iR;

            ComputeDescriptor(iVertexArray, NHull, R, d_, bd_, NS, iV, -1.0f);
        }
    }

    // Free memory.

    delete[] NHull.Element;
    delete[] NS;
    delete[] iV;
}

void VNClassifier::ComputeDescriptors(
    Array<int> iSurfelArray,
    Array<int> iVertexArray,
    float **pd,
    uchar **pbd)
{
    CRVLMem *pMem = &descriptorMem;

    int descriptorDataSize = sampledUnitSphere.h * SO3Samples.c;

    float *d;

    RVLMEM_ALLOC_STRUCT_ARRAY(pMem, float, descriptorDataSize, d);

    uchar *bd;

    RVLMEM_ALLOC_STRUCT_ARRAY(pMem, uchar, descriptorDataSize, bd);

    ComputeDescriptors(iSurfelArray, iVertexArray, d, bd);

    *pd = d;

    *pbd = bd;
}

void VNClassifier::ComputeDescriptors(
    float *dSrc,
    float *dTgt,
    uchar *bd)
{
    // Create CTI form dSrc.

    float tCTIc_CTI[3];

    vtkSmartPointer<vtkPolyData> polyData = GenerateCTIPrimitivePolydata_RW(sampledUnitSphere.Element, dSrc, sampledUnitSphere.h, false,
                                                                            NULL, tCTIc_CTI);

    // Copy CTI vertices to ptArray.

    Array<Point> ptArray;

    ptArray.n = polyData->GetNumberOfPoints();

    ptArray.Element = new Point[ptArray.n];

    vtkSmartPointer<vtkFloatArray> pointData = pointData->SafeDownCast(polyData->GetPoints()->GetData());

    Point *pPt = ptArray.Element;

    int iPt;
    QList<MeshEdgePtr> *pEdgeList;

    for (iPt = 0; iPt < ptArray.n; iPt++, pPt++)
    {
        pointData->GetTypedTuple(iPt, pPt->P);

        RVLSUM3VECTORS(pPt->P, tCTIc_CTI, pPt->P);

        pEdgeList = &(pPt->EdgeList);

        RVLQLIST_INIT(pEdgeList);
    }

    // Copy CTI faces to faces.

    vtkSmartPointer<vtkCellArray> faceData = polyData->GetPolys();

    int nFaces = polyData->GetNumberOfPolys();

    int nFacePtsTotal = 0;

    int iFace;
    vtkIdType *ptIdx;
    vtkIdType nFacePts;
    faceData->InitTraversal();

    for (iFace = 0; iFace < nFaces; iFace++)
    {
        faceData->GetNextCell(nFacePts, ptIdx);

        nFacePtsTotal += (int)nFacePts;
    }

    Array<Array<int>> faces;

    faces.Element = new Array<int>[nFaces];

    int *ptIdxMem = new int[nFacePtsTotal];

    int *iPtIdx = ptIdxMem;

    faceData->InitTraversal();

    int i;

    for (iFace = 0; iFace < nFaces; iFace++)
    {
        faceData->GetNextCell(nFacePts, ptIdx);

        faces.Element[iFace].n = (int)nFacePts;
        faces.Element[iFace].Element = iPtIdx;

        for (i = 0; i < nFacePts; i++)
            *(iPtIdx++) = (int)(ptIdx[i]);
    }

    // Create edges.

    MeshEdge *edgeMem = new MeshEdge[nFacePtsTotal];

    MeshEdge *pEdge = edgeMem;

    MeshEdgePtr *edgePtrMem = new MeshEdgePtr[2 * nFacePtsTotal];

    MeshEdgePtr *pEdgePtr = edgePtrMem;

    bool *bEdge = new bool[ptArray.n * ptArray.n];

    memset(bEdge, 0, ptArray.n * ptArray.n * sizeof(bool));

    // Only for debugging purpose!

    // int nDebug = 0;

    // int *edgeCnt = new int[ptArray.n * ptArray.n];

    // memset(edgeCnt, 0, ptArray.n * ptArray.n * sizeof(int));

    ///

    Array<int> *pFace;
    int iPt1, iPt2;
    int iEdge;

    for (iFace = 0; iFace < nFaces; iFace++)
    {
        pFace = faces.Element + iFace;

        iPt1 = pFace->Element[pFace->n - 1];

        for (i = 0; i < pFace->n; i++)
        {
            // nDebug++;

            iPt2 = pFace->Element[i];

            // edgeCnt[iPt1 * ptArray.n + iPt2]++;

            iEdge = iPt1 * ptArray.n + iPt2;

            // if (iPt1 == 0 || iPt2 == 0)
            //	int debug = 0;

            if (!bEdge[iEdge])
            {
                // if (bEdge[iPt1 * ptArray.n + iPt2])
                //	int debug = 0;

                bEdge[iEdge] = bEdge[iPt2 * ptArray.n + iPt1] = true;

                pEdge->iVertex[0] = iPt1;
                pEdge->iVertex[1] = iPt2;
                pEdge->pVertexEdgePtr[0] = pEdgePtr;
                pEdge->pVertexEdgePtr[1] = pEdgePtr + 1;

                pPt = ptArray.Element + iPt1;

                pEdgeList = &(pPt->EdgeList);

                RVLQLIST_ADD_ENTRY(pEdgeList, pEdgePtr);

                pEdgePtr->pEdge = pEdge;

                pEdgePtr++;

                pPt = ptArray.Element + iPt2;

                pEdgeList = &(pPt->EdgeList);

                RVLQLIST_ADD_ENTRY(pEdgeList, pEdgePtr);

                pEdgePtr->pEdge = pEdge;

                pEdgePtr++;

                pEdge++;
            }

            iPt1 = iPt2;
        }
    }

    // Only for debugging purpose!

    // for (i = 0; i < ptArray.n * ptArray.n; i++)
    //	if (edgeCnt[i] > 1)
    //		int debug = 0;

    // for (i = 0; i < ptArray.n; i++)
    //	for (int j = i + 1; j < ptArray.n; j++)
    //		if (edgeCnt[i * ptArray.n + j] > 0)
    //			if (edgeCnt[j * ptArray.n + i] == 0)
    //				int debug = 0;

    // delete[] edgeCnt;

    ///

    delete[] bEdge;

    // Compute a CTI descriptor for every rotation in SO3Samples.

    int iR, j;
    float *d;
    float *R;
    float *N;
    float N_[3];
    float dmax, d_;
    Point *pPtNext, *pPt_;
    uchar *bd_;

    for (iR = 0; iR < SO3Samples.c; iR++)
    {
        R = SO3Samples.Element + 9 * iR;

        d = dTgt + sampledUnitSphere.h * iR;

        bd_ = bd + sampledUnitSphere.h * iR;

        for (i = 0; i < sampledUnitSphere.h; i++)
        {
            // if (i == 33)
            //	int debug = 0;

            N = sampledUnitSphere.Element + 3 * i;

            RVLMULMX3X3VECT(R, N, N_);

            pPt = ptArray.Element;

            dmax = RVLDOTPRODUCT3(N_, pPt->P);

            for (j = 1; j < ptArray.n; j++)
            {
                pPt_ = ptArray.Element + j;

                d_ = RVLDOTPRODUCT3(N_, pPt_->P);

                if (d_ > dmax)
                    dmax = d_;
            }

            // while (pPt)
            //{
            //	pPtNext = NULL;

            //	pEdgePtr = pPt->EdgeList.pFirst;

            //	while (pEdgePtr)
            //	{
            //		iPt2 = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr);

            //		pPt_ = ptArray.Element + iPt2;

            //		d_ = RVLDOTPRODUCT3(N_, pPt_->P);

            //		if (d_ > dmax)
            //		{
            //			dmax = d_;

            //			pPtNext = pPt_;

            //			break;
            //		}

            //		pEdgePtr = pEdgePtr->pNext;
            //	}

            //	pPt = pPtNext;
            //}

            // if (RVLABS(dmax_ - dmax) > 0.005f)
            //	int debug = 0;

            d[i] = dmax;

            bd_[i] = (dmax < 0.0f ? 1 : 0);
        }
    }

    // Deallocate memory.

    delete[] ptArray.Element;
    delete[] ptIdxMem;
    delete[] faces.Element;
    delete[] edgeMem;
    delete[] edgePtrMem;
}

void VNClassifier::ComputeDescriptors(
    Mesh *pConvexHull,
    float *R,
    float *d,
    uchar *bd,
    Point **pVertex)
{
    Point *pPt0 = pConvexHull->NodeArray.Element + pConvexHull->iValidVertices.Element[0];

    float d0 = -pPt0->P[2];

    int j;
    Point *pPt;
    float *P;

    for (j = 1; j < pConvexHull->iValidVertices.n; j++)
    {
        pPt = pConvexHull->NodeArray.Element + pConvexHull->iValidVertices.Element[j];

        P = pPt->P;

        if (-P[2] > d0)
        {
            d0 = -P[2];

            pPt0 = pPt;
        }
    }

    Pair<int, int> *tree = CTIDescriptorMapingData.tree;

    int rollBlockSize = 4 * CTIDescriptorMapingData.nRoll;

    pVertex[tree[0].a] = pPt0;

    int iChild, id, iRoll, iPt2, iD;
    float *NM;
    float *R__;
    float NS[3];
    float dmax, d_;
    Point *pPtNext, *pPt_;
    MeshEdgePtr *pEdgePtr;

    for (j = 0; j < rollBlockSize; j++)
    {
        pPt = pVertex[tree[j].a];

        iChild = tree[j].b;

        id = iChild % sampledUnitSphere.h;

        iRoll = iChild / sampledUnitSphere.h;

        NM = sampledUnitSphere.Element + 3 * id;

        R__ = R + 9 * iRoll;

        RVLMULMX3X3VECT(R__, NM, NS);

        dmax = RVLDOTPRODUCT3(NS, pPt->P);

        pPtNext = pPt;

        while (pPtNext)
        {
            pPtNext = NULL;

            pEdgePtr = pPt->EdgeList.pFirst;

            while (pEdgePtr)
            {
                iPt2 = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr);

                pPt_ = pConvexHull->NodeArray.Element + iPt2;

                if (pPt_->bValid)
                {
                    d_ = RVLDOTPRODUCT3(NS, pPt_->P);

                    if (d_ > dmax)
                    {
                        dmax = d_;

                        pPtNext = pPt_;

                        break;
                    }
                }

                pEdgePtr = pEdgePtr->pNext;
            }

            if (pPtNext)
                pPt = pPtNext;
        }

        // Only for debugging purpose!

        // float dmax_ = -1e9;

        // for (i = 0; i < pConvexHull->iValidVertices.n; i++)
        //{
        //	P = pConvexHull->NodeArray.Element[pConvexHull->iValidVertices.Element[i]].P;

        //	d_ = RVLDOTPRODUCT3(NS, P);

        //	if (d_ > dmax_)
        //		dmax_ = d_;
        //}

        // if (dmax_ != dmax)
        //	int debug = 0;

        ///

        iD = iRoll * sampledUnitSphere.h + id;

        d[iD] = dmax;

        bd[iD] = (dmax < 0.0f ? 1 : 0);

        pVertex[iChild] = pPt;
    } // for (j = 0; j < nRollBlock; j++)
}

bool VNClassifier::ComputeDescriptors(
    Array<int> iVertexArray,
    float *R,
    float *d,
    uchar *bd,
    bool bConvexHullVertexMemAllocated)
{
    // Copy vertices from pSurfels->vertexArray to convexHullVertices.

    if (!bConvexHullVertexMemAllocated)
        convexHullVertices.Element = new float[3 * iVertexArray.n];

    int i;
    float *P, *P_;

    for (i = 0; i < iVertexArray.n; i++)
    {
        P = pSurfels->vertexArray.Element[iVertexArray.Element[i]]->P;

        P_ = convexHullVertices.Element + 3 * i;

        RVLCOPY3VECTOR(P, P_);
    }

    convexHullVertices.h = iVertexArray.n;

    // Compute convex hull

    if (!convexHull.ConvexHull(convexHullVertices, &memConvexHull, true, false))
    {
        if (!bConvexHullVertexMemAllocated)
            delete[] convexHullVertices.Element;

        return false;
    }

    // Compute canonical orientation for convexHull.

    CanonicalOrientation(R);

    // Compute descriptor for the first canonical orientation.

    ComputeDescriptor(&convexHull, R, d, bd, &iVertexArray);

    // Map the computed descriptor to the descriptors of the other canonical orientations.

    float *d_ = d + sampledUnitSphere.h;
    uchar *bd_ = bd + sampledUnitSphere.h;

    int iR;
    int *iD;

    for (iR = 1; iR < alignment.CTIDescMap.nCTI; iR++, d_ += sampledUnitSphere.h, bd_ += sampledUnitSphere.h)
    {
        iD = alignment.CTIDescMap.iD + sampledUnitSphere.h * iR;

        for (i = 0; i < sampledUnitSphere.h; i++)
        {
            d_[i] = d[iD[i]];
            bd_[i] = bd[iD[i]];
        }
    }

    if (!bConvexHullVertexMemAllocated)
        delete[] convexHullVertices.Element;

    return true;
}

void VNClassifier::CTIs(
    Mesh *pMesh,
    int iModel)
{
    // Detect surfels.

    pSurfels->Init(pMesh);

    pSurfelDetector->Init(pMesh, pSurfels, pMem);

    printf("Segmentation to surfels...");

    pSurfelDetector->Segment(pMesh, pSurfels);

    printf("completed.\n");

    int nSurfels = pSurfels->NodeArray.n;

    printf("No. of surfels = %d\n", nSurfels);

    pSurfels->DetectVertices(pMesh);

    // Cluster surfels into convex and concave surfaces.

    bool bTorus;

    bTorus = false;

    int nSCClusters, nSUClusters;

    RVL_DELETE_ARRAY(SClusters.Element);

    RVL_DELETE_ARRAY(clusterMap);

    clusterMap = new int[pSurfels->NodeArray.n];

    PSGM_::ConvexAndConcaveClusters(pMesh, pSurfels, pSurfelDetector, &convexClustering, &concaveClustering,
                                    SClusters, nSCClusters, nSUClusters, convexClustering.minClusterSize, clusterMap, maxnSCClusters, maxnSUClusters, bConcavity, (bTorus ? maxnSTClusters : 0),
                                    visualizationData.bVisualizeConvexClusters, visualizationData.bVisualizeConcaveClusters, visualizationData.bVisualizeSurfels, visualizationData.bVisualizeAllClusters);

    printf("No. of clusters = %d\n", SClusters.n);

    // Create objects from clusters.

    RECOG::CreateObjectGraphFromSceneClusters(SClusters, pObjects);

    pObjects->GetVertices();

    /// Create descriptors from segments stored in pObjects.

    // QList<PSGM_::ModelInstance> CTIList;

    // QList<PSGM_::ModelInstance> *pCTIList = &CTIList;

    // RVLQLIST_INIT(pCTIList);

    FILE *fpDescriptors = fopen(modelDataBase, "a");

    Init(pMesh);

    if (visualizationData.bVisualizeZAxes)
        visualizationData.ZAxes = new Array<OrientedPoint>[pObjects->objectArray.n + 1];

    // NHull is not used
    Array<SURFEL::NormalHullElement> NHull; // not used
    NHull.Element = NULL;
    NHull.n = 0;

    float *d = new float[sampledUnitSphere.h];
    uchar *bd = new uchar[sampledUnitSphere.h];
    float *NS = new float[sampledUnitSphere.h * sampledUnitSphere.w];
    int *iV = new int[sampledUnitSphere.h];

    int i, iSegment, iRF, type, nV;
    SURFEL::Object *pSegment;
    Array<Pose3D> referenceFrames;
    // PSGM_::ModelInstance *pCTI;
    Array<OrientedPoint> *pZAxes;
    float o;
    float *R;

    for (iSegment = 0; iSegment < pObjects->objectArray.n; iSegment++)
    {
        pSegment = pObjects->objectArray.Element + iSegment;

        pSurfels->GetVertices(pSegment->iVertexArray, convexHullVertices);

        // Create convex hull of the segments in segments.

        if (!convexHull.ConvexHull(convexHullVertices, &memConvexHull))
            continue;

        // Define reference frames.

        pZAxes = (visualizationData.bVisualizeZAxes ? visualizationData.ZAxes + iSegment : NULL);

        RECOG::ReferenceFrames(&convexHull, referenceFrames, kFaceClusterSizeThr, false, pZAxes);

        // Compute descriptors.

        for (iRF = 0; iRF < referenceFrames.n; iRF++)
        {
            R = referenceFrames.Element[iRF].R;
            ComputeDescriptor(pSegment, NHull, R, d, bd, NS, iV, true, false);

            if (bDepthImage)
                SaveDescriptor(fpDescriptors, d, bd, R, iModel, 0, iSegment, (pSegment->flags & RVLPCSEGMENT_OBJECT_FLAG_CONCAVE ? 1 : 0));
            else
            {
                nV = 0;

                for (i = 0; i < sampledUnitSphere.h; i++)
                    if (bd[i])
                        nV++;

                if (nV >= sampledUnitSphere.h * 50 / 100)
                    SaveDescriptor(fpDescriptors, d, bd, R, iModel, 0, iSegment, (pSegment->flags & RVLPCSEGMENT_OBJECT_FLAG_CONCAVE ? 1 : 0));
                else
                    int debug = 0;
            }
        }

        //

        delete[] referenceFrames.Element;
    }

    ///

    // Create descriptors from the convex hull of the whole object.

    if (!bDepthImage && pObjects->objectArray.n > 1)
    {
        pSurfels->GetVertices(convexHullVertices);

        if (convexHull.ConvexHull(convexHullVertices, &memConvexHull))
        {
            pZAxes = (visualizationData.bVisualizeZAxes ? visualizationData.ZAxes + iSegment : NULL);

            RECOG::ReferenceFrames(&convexHull, referenceFrames, kFaceClusterSizeThr, false, pZAxes);

            for (iRF = 0; iRF < referenceFrames.n; iRF++)
            {
                R = referenceFrames.Element[iRF].R;
                for (i = 0; i < pSurfels->vertexArray.n; i++)
                    iVertexArray.Element[i] = i;
                iVertexArray.n = pSurfels->vertexArray.n;
                ComputeDescriptor(iVertexArray, NHull, R, d, bd, NS, iV, 1.0f, false, false);
                SaveDescriptor(fpDescriptors, d, bd, R, iModel, 0, iSegment);
            }

            delete[] referenceFrames.Element;
        }
    }

    fclose(fpDescriptors);

    ///

    // Visualization.

    if (visualizationData.bVisualizeZAxes)
    {
        uchar SelectionColor[] = {0, 255, 0};

        RandomColors(SelectionColor, visualizationData.clusterColor, pObjects->objectArray.n);

        InitDisplay(visualizationData.pVisualizer, pMesh, SelectionColor);

        pSurfels->DisplayEdgeFeatures();

        DisplayClusters();

        Box<float> BBox;

        pMesh->BoundingBox(&BBox);

        visualizationData.meshSize = BoxSize<float>(&BBox);

        visualizationData.pVisualizer->Run();

        visualizationData.pVisualizer->renderer->RemoveAllViewProps();

        for (iSegment = 0; iSegment < pObjects->objectArray.n; iSegment++)
            delete[] visualizationData.ZAxes[iSegment].Element;

        delete[] visualizationData.ZAxes;
    }

    delete[] d;
    delete[] bd;
    delete[] NS;
    delete[] iV;
}

void VNClassifier::ComputeDescriptor(
    Array<int> iVertexArray,
    Array<SURFEL::NormalHullElement> NHull,
    float *R,
    float *d,
    uchar *bd,
    float *NS,
    int *iV,
    float o,
    bool bNormalTest,
    bool bVisibilityTest,
    float *J)
{
    float dmax = 0.0f;
    float dmaxValid = 0.0f;

    int i;
    int iN;
    int iVertex;
    float *N_, *P;
    float *N;
    SURFEL::Vertex *pVertex;
    // SURFEL::Vertex *pTangentVertex;
    // float *d_;
    // uchar *bd_;
    float d__, distFromNHull;
    // uchar bd__;
    bool bdmax, bdmaxValid;

    for (iN = 0; iN < sampledUnitSphere.h; iN++)
    {
        // if (iN == 33)
        //	int debug = 0;

        N_ = sampledUnitSphere.Element + sampledUnitSphere.w * iN;

        N = NS + sampledUnitSphere.w * iN;

        RVLMULMX3X3VECT(R, N_, N);

        bdmax = bdmaxValid = false;

        for (i = 0; i < iVertexArray.n; i++)
        {
            iVertex = iVertexArray.Element[i];

            pVertex = pSurfels->vertexArray.Element[iVertex];

            P = pVertex->P;

            d__ = RVLDOTPRODUCT3(N, P);

            if (bdmax)
            {
                if (o * d__ > o * dmax)
                {
                    dmax = d__;

                    iV[iN] = iVertex;
                }
            }
            else
            {
                dmax = d__;

                iV[iN] = iVertex;

                bdmax = true;
            }

            if (bNormalTest)
            {
                if (d__ < 0.0f || !bVisibilityTest)
                {
                    // if (pVertex->normalHull.n >= 3)
                    {
                        distFromNHull = pSurfels->DistanceFromNormalHull(pVertex->normalHull, N);

                        if (distFromNHull <= maxDistFromNHull)
                        {
                            if (bdmaxValid)
                            {
                                if (o * d__ > o * dmaxValid)
                                    dmaxValid = d__;
                            }
                            else
                            {
                                dmaxValid = d__;

                                bdmaxValid = true;
                            }
                        }
                    }
                }
            }
        } // for (i = 0; i < iVertexArray.n; i++)

        if (bNormalTest)
        {
            if (bdmaxValid)
                d[iN] = ((bd[iN] = (o * (dmax - dmaxValid) <= maxedmax)) ? dmaxValid : dmax);
            else
            {
                d[iN] = dmax;

                bd[iN] = 0;
            }
        }
        else
        {
            d[iN] = dmax;

            bd[iN] = 1;
        }

        // bd__ = 0x00;

        // if (pTangentVertex->normalHull.n >= 3)
        //{
        //	distFromNHull = pSurfels->DistanceFromNormalHull(pTangentVertex->normalHull, N);

        //	if (distFromNHull <= 0.0f)
        //		bd__ = 0x01;
        //}

        // if (bd__ == 0x00)
        //{
        //	distFromNHull = pSurfels->DistanceFromNormalHull(NHull, N);

        //	bd__ = (distFromNHull <= maxDistFromNHull);
        //}

        // bd[iN] = bd__;
    }

    if (J)
    {
    }
}

void VNClassifier::ComputeDescriptor(
    SURFEL::Object *pSegment,
    Array<SURFEL::NormalHullElement> NHull,
    float *R,
    float *d,
    uchar *bd,
    float *NS,
    int *iV,
    bool bNormalTest,
    bool bVisibilityTest,
    float *J)
{
    ComputeDescriptor(pSegment->iVertexArray, NHull, R, d, bd, NS, iV,
                      (pSegment->flags & RVLPCSEGMENT_OBJECT_FLAG_CONCAVE ? -1.0f : 1.0f), bNormalTest, bVisibilityTest);
}

void VNClassifier::ComputeDescriptor(
    Mesh *pConvexHull,
    float *R,
    float *d,
    uchar *bd,
    Array<int> *piVertexArray,
    float o,
    int *iV)
{
    int i, iN;
    float *NM, *P;
    float NS[3];
    float dmax, d_;
    bool bFirst;

    if (piVertexArray)
    {
        dmax = 0.0f;
        float dmaxValid = 0.0f;

        int iVertex;
        SURFEL::Vertex *pVertex;
        float d__;
        bool bdmax, bdmaxValid;

        for (iN = 0; iN < sampledUnitSphere.h; iN++)
        {
            // if (iN == 22)
            //	int debug = 0;

            NM = sampledUnitSphere.Element + sampledUnitSphere.w * iN;

            RVLMULMX3X3VECT(R, NM, NS);

            bdmax = bdmaxValid = false;

            for (i = 0; i < pConvexHull->iValidVertices.n; i++)
            {
                iVertex = piVertexArray->Element[pConvexHull->iValidVertices.Element[i]];

                // for (i = 0; i < piVertexArray->n; i++)
                //{
                //	iVertex = piVertexArray->Element[i];

                pVertex = pSurfels->vertexArray.Element[iVertex];

                P = pVertex->P;

                d__ = RVLDOTPRODUCT3(NS, P);

                if (bdmax)
                {
                    if (o * d__ > o * dmax)
                    {
                        dmax = d__;

                        if (iV)
                            iV[iN] = iVertex;
                    }
                }
                else
                {
                    dmax = d__;

                    if (iV)
                        iV[iN] = iVertex;

                    bdmax = true;
                }

                if (piVertexArray)
                {
                    if (d__ < 0.0f)
                    {
                        if (!(pVertex->type & RVLSURFELVERTEX_TYPE_OCCLUSION))
                        {
                            if (bdmaxValid)
                            {
                                if (o * d__ > o * dmaxValid)
                                    dmaxValid = d__;
                            }
                            else
                            {
                                dmaxValid = d__;

                                bdmaxValid = true;
                            }
                        }
                    }
                }
            } // for every vertex

            if (bdmaxValid)
                d[iN] = ((bd[iN] = (o * (dmax - dmaxValid) <= maxedmax)) ? dmaxValid : dmax);
            else
            {
                d[iN] = dmax;

                bd[iN] = 0x00;
            }
        } // for every component of the convex template
    }
    else
    {
        for (iN = 0; iN < sampledUnitSphere.h; iN++)
        {
            NM = sampledUnitSphere.Element + sampledUnitSphere.w * iN;

            RVLMULMX3X3VECT(R, NM, NS);

            bFirst = true;

            for (i = 0; i < pConvexHull->iValidVertices.n; i++)
            {
                P = pConvexHull->NodeArray.Element[pConvexHull->iValidVertices.Element[i]].P;

                d_ = RVLDOTPRODUCT3(NS, P);

                if (bFirst)
                {
                    dmax = d_;
                    bFirst = false;
                }
                else if (d_ > dmax)
                    dmax = d_;
            }

            d[iN] = dmax;
            bd[iN] = (dmax < 0.0f ? 1 : 0);
        }
    }
}

void VNClassifier::MergeDescriptors(
    float *dSrc1,
    uchar *bdSrc1,
    float *dSrc2,
    uchar *bdSrc2,
    float o,
    float *dTgt,
    uchar *bdTgt)
{
    float *dSrc1_ = dSrc1;
    float *dSrc2_ = dSrc2;
    float *dTgt_ = dTgt;

    uchar *bdSrc1_ = bdSrc1;
    uchar *bdSrc2_ = bdSrc2;
    uchar *bdTgt_ = bdTgt;

    int i, j;

    for (i = 0; i < SO3Samples.c; i++)
    {
        for (j = 0; j < sampledUnitSphere.h; j++)
        {
            if (o * dSrc1_[j] >= o * dSrc2_[j])
            {
                dTgt_[j] = dSrc1_[j];
                bdTgt_[j] = bdSrc1_[j];
            }
            else
            {
                dTgt_[j] = dSrc2_[j];
                bdTgt_[j] = bdSrc2_[j];
            }
        }

        dSrc1_ += sampledUnitSphere.h;
        dSrc2_ += sampledUnitSphere.h;
        dTgt_ += sampledUnitSphere.h;

        bdSrc1_ += sampledUnitSphere.h;
        bdSrc2_ += sampledUnitSphere.h;
        bdTgt_ += sampledUnitSphere.h;
    }
}

void VNClassifier::MergeDescriptors(
    float *dSrc1,
    float *dSrc2,
    float o,
    float *dTgt)
{
    float *dSrc1_ = dSrc1;
    float *dSrc2_ = dSrc2;
    float *dTgt_ = dTgt;

    int i, j;

    for (j = 0; j < sampledUnitSphere.h; j++)
    {
        if (o * dSrc1_[j] >= o * dSrc2_[j])
        {
            dTgt_[j] = dSrc1_[j];
        }
        else
        {
            dTgt_[j] = dSrc2_[j];
        }
    }
}

void VNClassifier::Learn(
    char *modelSequenceFileName,
    int iClass,
    int iMM,
    Visualizer *pVisualizer)
{
    FILE *fpSaveSuperSegments = NULL;
    if (PartSuperSegmentID)
    {
        fpSaveSuperSegments = fopen(PartSuperSegmentID, "w");
        fclose(fpSaveSuperSegments);
    }
    FILE *fpDescriptors = fopen("C:\\RVL\\ExpRez\\descriptors.txt", "w");
    FILE *fpVertices = fopen("C:\\RVL\\ExpRez\\vertices.txt", "w");
    FILE *fCorrespondences = fopen("C:\\RVL\\ExpRez\\Correspondences.txt", "w");
    FILE *fpDescriptorsMerged = fopen("C:\\RVL\\ExpRez\\descriptorsMerged.txt", "w");
    FILE *fPrimitiveClass = fopen((std::string(resultsFolder) + "\\primitiveClass.txt").data(), "w");

    fclose(fpDescriptorsMerged);
    fclose(fpDescriptors);
    fclose(fpVertices);
    fclose(fCorrespondences);
    fclose(fPrimitiveClass);

    float resolution = 0.01f;

    Eigen::MatrixXf A(3, 66);

    A = alignment.ConvexTemplatenT();

    // int iMetaModel = (iClass < classArray.n ? classArray.Element[iClass].iMetaModel : -1);
    int iMetaModel = (iMM < classArray.n ? classArray.Element[iMM].iMetaModel : -1);
    ;

    Mesh mesh;

    FileSequenceLoader modelsLoader;
    FileSequenceLoader dbLoader;

    char modelFilePath[200];
    char modelFileName[200];

    if (!modelDataBase)
        modelDataBase = "modelDB.dat";

    if (!modelsInDataBase)
        modelsInDataBase = "DBModels.txt";

    modelsLoader.Init(modelSequenceFileName);
    dbLoader.Init(modelsInDataBase);

    FILE *fpConvexHullDataBase = NULL;

    bool saveDBSequenceFile = true;

    if (trainingMethod == 4)
    {
        // FILE *fpCTIDescriptors = fopen(modelDataBase, "w");

        // fclose(fpCTIDescriptors);

        saveDBSequenceFile = true;

        char *CHDataBaseFileName = RVLCreateFileName(modelDataBase, ".dat", -1, ".ch.dat");

        fpConvexHullDataBase = fopen(CHDataBaseFileName, "w");

        delete[] CHDataBaseFileName;
    }

    printf("Model DB creation started...\n");

    unsigned char color[] = {0, 128, 255};

    float *dS = NULL;
    bool *bdS = NULL;

    int currentModelID;
    Box<float> SBoundingBox;
    VN *pModel;
    float R[9];
    float t[3];
    char modelFilePath_[200];
    char modelFileName_[200];
    char *modelDescriptorFileName;
    int modelID, modelID_;
    FILE *fpDescriptor;

    if (bLearnMetamodels)
    {
        int nMetamodels;
        FILE *fpMetamodelCost = fopen("MetamodelCost.txt", "w");

        nMetamodels = 5;
        int MetaModelIDsArray[] = {0, 2, 3, 6, 8};
        float MetaModelScore[] = {0, 0, 0, 0, 0};
        float temp;

        while (modelsLoader.GetNext(modelFilePath, modelFileName))
        {
            if (ModelExistInDB(modelFilePath, dbLoader))
                continue;

            printf("\nProcessing model %s!\n", modelFileName);

            if (LoadMesh(vpMeshBuilder, modelFilePath, &mesh, false, NULL, NULL))
            {
                FILE *fp = fopen("C:\\RVL\\Debug\\P.txt", "w");

                int i;
                float *P;

                for (i = 0; i < mesh.NodeArray.n; i++)
                {
                    P = mesh.NodeArray.Element[i].P;

                    fprintf(fp, "%f\t%f\t%f\n", P[0], P[1], P[2]);
                }

                fclose(fp);

                // Check if there exists a label file.

                char *labelFileName = RVLCreateFileName(modelFilePath, ".ply", -1, "_.seg");

                mesh.LoadLabels(labelFileName);

                delete[] labelFileName;

                currentModelID = dbLoader.GetLastModelID() + 1;

                pMem->Clear();

                dbLoader.ResetID();

                modelID = -1;

                // Determine the model ID of the model defined in file modelFileName.

                while (dbLoader.GetNext(modelFilePath_, modelFileName_, &modelID_))
                    if (strcmp(modelFileName, modelFileName_) == 0)
                    {
                        modelID = modelID_;

                        break;
                    }

                // If the model is not found, then R <- unit matrix and t <- null vector, otherwise align model with the reference model of this class.

                if (modelID < 0)
                {
                    printf("CTI of the considered model is not available!\n");

                    RVLUNITMX3(R);
                    RVLNULL3VECTOR(t);
                }
                else
                {
                    printf("CTI alignment.\n");

                    alignment.ObjectAlignment(alignment.MCTISet.SegmentCTIs.Element[modelID],
                                              alignment.MCTISet.pCTI.Element,
                                              alignment.MCTISet.SegmentCTIs.Element[classArray.Element[iClass].iRefInstance],
                                              alignment.MCTISet.pCTI.Element, A, R, t, true);
                }

                if (trainingMethod == 0)
                {

                    std::vector<std::vector<float>> metaModelScorePerModelArr;
                    float score = -1;

                    for (int iMM = 0; iMM < nMetamodels; iMM++)
                    // for (int iMM = 0; iMM < 1; iMM++)
                    {
                        // metaModelScorePerModelArr[iMM].push_back(ComputeDescriptor(&mesh, R, t, dS, bdS, SBoundingBox, MetaModelIDsArray[i], score));
                        temp = ComputeDescriptor(&mesh, R, t, dS, bdS, SBoundingBox, MetaModelIDsArray[iMM], score);
                        MetaModelScore[iMM] += temp;
                        fprintf(fpMetamodelCost, "%f\t", temp);

                        if (bVisualization)
                        {
                            if (pVisualizer)
                            {
                                pModel = models[MetaModelIDsArray[iMM]];

                                pVisualizer->renderer->RemoveAllViewProps();

                                pModel->Display(pVisualizer, 0.01f, dS, bdS, visualizationData.SDFSurfaceValue);

                                pVisualizer->Run();
                            }
                        }
                    }
                    fprintf(fpMetamodelCost, "\n");
                }
            }
        } // for every model on the list modelSequenceFileName

        fprintf(fpMetamodelCost, "%f\t%f\t%f\t%f\t%f\n", MetaModelScore[0], MetaModelScore[1], MetaModelScore[2], MetaModelScore[3], MetaModelScore[4]);

        float min = MetaModelScore[0];
        int idMin = MetaModelIDsArray[0];
        for (int iMM = 1; iMM < nMetamodels; iMM++)
        {
            if (MetaModelScore[iMM] < min)
            {
                min = MetaModelScore[iMM];
                idMin = MetaModelIDsArray[iMM];
            }
        }
        fclose(fpMetamodelCost);
        iMetaModel = idMin;

        modelsLoader.ResetID();
    } // if (bLearnMetamodels)

    memConvexHull.Create(10000000);

    sceneID = 0;

    QList<VNInstance> *pModelVNList = &modelVNList;

    VNInstance VNSceneInstance;

    if (bGenerateModelVNInstances)
    {
        FILE *fpDB = fopen(modelDataBase, "a");

        char *VNModelDataBaseFileName = RVLCreateFileName(modelDataBase, ".dat", -1, ".vn.dat");

        FILE *fpVNDB = fopen(VNModelDataBaseFileName, "w");

        delete[] VNModelDataBaseFileName;

        RVLQLIST_INIT(pModelVNList);

        while (modelsLoader.GetNext(modelFilePath, modelFileName))
        {

            if (ModelExistInDB(modelFilePath, dbLoader))
                continue;

            SetSceneFileName(modelFileName);

            // Process mesh.

            // mesh.LoadPolyDataFromPLY(modelFilePath);

            // vtkSmartPointer<vtkPolyData> polyData = MESH::CreateVisibleSurfaceMesh(mesh.pPolygonData, 0.01f, 5);

            //// Create a mapper and actor.
            // vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            // mapper->SetInputData(polyData);
            // vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
            // actor->SetMapper(mapper);

            // pVisualizer->renderer->AddActor(actor);

            // pVisualizer->Run();

            ///

            // if (ModelExistInDB(modelFileName, dbLoader))
            //	continue;

            printf("\nProcessing model %s!\n", modelFileName);

            // saveDBSequenceFile = true;

            if (LoadMesh(vpMeshBuilder, modelFilePath, &mesh, false, NULL, NULL))
            {
                // FILE *fp = fopen("C:\\RVL\\Debug\\P.txt", "w");

                // int i;
                // float *P;

                // for (i = 0; i < mesh.NodeArray.n; i++)
                //{
                //	P = mesh.NodeArray.Element[i].P;

                //	fprintf(fp, "%f\t%f\t%f\n", P[0], P[1], P[2]);
                //}

                // fclose(fp);

                // Load labels.

                bool bLabels = false;
                if (bLabels)
                {
                    char *labelFileName = RVLCreateFileName(modelFilePath, ".ply", -1, "_.seg");

                    mesh.LoadLabels(labelFileName);

                    delete[] labelFileName;
                }
                // currentModelID <- model ID of the new model

                currentModelID = dbLoader.GetLastModelID() + 1;

                // Clear pMem.

                pMem->Clear();

                // modelID <- model ID of the model defined in file modelFileName.

                dbLoader.ResetID();

                modelID = -1;

                while (dbLoader.GetNext(modelFilePath_, modelFileName_, &modelID_))
                    if (strcmp(modelFileName, modelFileName_) == 0)
                    {
                        modelID = modelID_;

                        break;
                    }

                // If the model is not found, then R <- unit matrix and t <- null vector, otherwise align model with the reference model of this class.

                if (bVNCTIAlignment)
                {
                    if (modelID < 0)
                    {
                        printf("CTI of the considered model is not available!\n");

                        RVLUNITMX3(R);
                        RVLNULL3VECTOR(t);
                    }
                    else
                    {
                        printf("CTI alignment.\n");

                        alignment.ObjectAlignment(alignment.MCTISet.SegmentCTIs.Element[modelID],
                                                  alignment.MCTISet.pCTI.Element,
                                                  alignment.MCTISet.SegmentCTIs.Element[classArray.Element[iClass].iRefInstance],
                                                  alignment.MCTISet.pCTI.Element, A, R, t, true);
                    }
                }
                else
                {
                    RVLUNITMX3(R);
                    RVLNULL3VECTOR(t);
                }

                if (bPlusPlus)
                {
                    VNInstance VNI;

                    Primitives(&mesh, &VNI);
                }
                else
                {
                    if (trainingMethod == 0)
                    {

                        // ComputeDescriptor(&mesh, R, t, dS, bdS, SBoundingBox, iMetaModel);

                        ComputeDescriptor(&mesh, R, t, dS, bdS, SBoundingBox, iMetaModel);

                        dbLoader.AddModel(currentModelID, modelFilePath, modelFileName);

                        modelDescriptorFileName = RVLCreateFileName(modelFilePath, ".ply", -1, ".vnd");

                        fpDescriptor = fopen(modelDescriptorFileName, "w");

                        delete[] modelDescriptorFileName;

                        SaveDescriptor(fpDescriptor, dS, bdS, modelID_, iMetaModel);

                        SaveDescriptor(fpVNDB, dS, bdS, modelID_, iMetaModel);

                        fclose(fpDescriptor);

                        if (bVisualization)
                        {
                            if (pVisualizer)
                            {
                                pModel = models[iMetaModel];

                                // ExpandBox<float>(&SBoundingBox, 10.0f * resolution);

                                pVisualizer->renderer->RemoveAllViewProps();

                                // pModel->Display(pVisualizer, SBoundingBox, visualizationData.resolution, dS, bdS, visualizationData.SDFSurfaceValue);
                                pModel->Display(pVisualizer, 0.01f, dS, bdS, visualizationData.SDFSurfaceValue);

                                // pVisualizer->DisplayPointSet<float, MESH::Sample>(sceneObject.sampleArray, color, 6.0f);

                                pVisualizer->Run();
                            }
                        }

                        RVL_DELETE_ARRAY(dS);
                        RVL_DELETE_ARRAY(bdS);
                    }
                    else if (trainingMethod == 1 || trainingMethod == 3)
                    {
                        int i;

                        for (i = 0; i < modelSet.n; i++)
                            if (sceneID == modelSet.Element[i])
                                break;

                        // if (i < modelSet.n)
                        //{
                        VNInstance *pVNModel;

                        ComputeDescriptor(&mesh, R, t, dS, bdS, SBoundingBox, iMetaModel, pMem0, &pVNModel);

                        float *ptCompDist;

                        pVNModel->SparsePCDistanceToComponents(this, &mesh, modelFilePath, ptCompDist, true);

                        RVL_DELETE_ARRAY(ptCompDist);

                        if (bSaveSupersegments && visualizationData.iSelectedComponent != -1)
                        {

                            FILE *fpSaveSuperSegments = fopen(PartSuperSegmentID, "a");

                            fprintf(fpSaveSuperSegments, "%d\t%d\n", sceneID, visualizationData.iSelectedComponent);
                            fclose(fpSaveSuperSegments);
                        }

                        if (!bSaveSupersegments)
                        {
                            dbLoader.AddModel(currentModelID, modelFilePath, modelFileName);

                            QList<VNInstance> *pModelVNList = &modelVNList;

                            RVLQLIST_ADD_ENTRY(pModelVNList, pVNModel);

                            // Save descriptors to .dat file from object array
                            SURFEL::Object *pObject;
                            int clusterType;

                            for (int iObject = 0; iObject < pObjects->objectArray.n; iObject++)
                            {
                                pObject = pObjects->objectArray.Element + iObject;

                                if (RVL::CheckFlag(pObject->flags, RVLPCSEGMENT_OBJECT_FLAG_CONCAVE))
                                    clusterType = 1;
                                else
                                    clusterType = 0;

                                SaveDescriptor(fpDB, pObject->d, pObject->bd, R, currentModelID, pObject->label, iObject, clusterType);
                            }

                            // int *Correspondences;

                            // VNSceneInstance.Match(pModelVNList, Correspondences, this, &mesh);

                            // delete[] Correspondences;
                        }
                        //}
                    }
                    else if (trainingMethod == 2)
                    {
                        SegmentToParts(&mesh);
                    }
                    else if (trainingMethod == 4)
                    {
                        if (modelID < 0)
                        {
                            CTIs(&mesh, currentModelID);

                            dbLoader.AddModel(currentModelID, modelFilePath, modelFileName);

                            if (fpConvexHullDataBase)
                            {
                                pSurfels->GetVertices(convexHullVertices);

                                int i;
                                float *P;

                                for (i = 0; i < convexHullVertices.h; i++)
                                {
                                    P = convexHullVertices.Element + 3 * i;

                                    RVLSCALE3VECTOR(P, trainingModelScale, P);
                                }

                                if (convexHull.ConvexHull(convexHullVertices, &memConvexHull))
                                {
                                    fprintf(fpConvexHullDataBase, "%d\t%d\t%d\n", currentModelID, convexHull.iValidVertices.n, 0);

                                    convexHull.SaveConvexHullVertices(fpConvexHullDataBase);

                                    convexHull.CreateVTKPolyData();

                                    // Visualizer visualizer;

                                    // visualizer.Create();

                                    // visualizer.SetMesh(&mesh);

                                    // visualizer.AddMesh(convexHull.pPolygonData);

                                    // visualizer.Run();
                                }
                                else
                                    printf("Convex hull is not created!\n");
                            }
                        }
                    }
                }
            }
            else // Mesh cannot be loaded.
                printf("VN descriptor cannot be computed for this mesh!\n");

            sceneID++;
        } // for every model in modelSequenceFileName

        if (bSaveGeneratedVNInstances)
            SaveVNInstances(pModelVNList, this);

        fclose(fpVNDB);

        printf("Model DB creation completed!\n");

        if (saveDBSequenceFile)
            SaveModelID(dbLoader, modelsInDataBase);

        if (fpConvexHullDataBase)
            fclose(fpConvexHullDataBase);

        // fclose(fp);

        fclose(fpDB);

        RVL_DELETE_ARRAY(modelVNArray.Element);
        modelVNArray.Element = new VNInstance *[sceneID];
        QLIST::CreatePtrArray<VNInstance>(pModelVNList, &modelVNArray);
    } // if (bGenerateModelVNInstances)

    if (bSampleModelPointClouds)
    {
        FILE *fpDB = fopen(modelDataBase, "a");

        int iModel = 0;

        int i, iPt;
        float NProx, maxNProx;
        float *P, *N;
        char *modelPCFileName;
        Array3D<QList<QLIST::Index>> grid;
        Box<double> boundingBox;
        Array<int> voxels;
        Array<OrientedPoint> PC;
        VNInstance *pModel;
        OrientedPoint *pPt;

        while (modelsLoader.GetNext(modelFilePath, modelFileName))
        {
            // char *modelPCFileName = RVLCreateFileName(modelFilePath, ".ply", -1, ".pti");

            // FILE *fpPC = fopen(modelPCFileName, "w");

            // delete[] modelPCFileName;

            if (iModel == 836)
            {
                if (LoadMesh(vpMeshBuilder, modelFilePath, &mesh, false, NULL, NULL))
                {
                    mesh.VoxelGridFilter(0.025f, 0.866f, PC, grid, boundingBox, voxels);

                    delete[] grid.Element;
                    delete[] voxels.Element;

                    pModel = modelVNArray.Element[iModel];

                    pModel->PC.Element = new float[3 * PC.n];
                    pModel->PC.w = 3;
                    pModel->PC.h = PC.n;
                    pModel->iCTIElement = new int[PC.n];

                    for (iPt = 0; iPt < PC.n; iPt++)
                    {
                        pPt = PC.Element + iPt;

                        P = pModel->PC.Element + 3 * iPt;

                        RVLCOPY3VECTOR(pPt->P, P);

                        maxNProx = -2.0f;

                        for (i = 0; i < sampledUnitSphere.h; i++)
                        {
                            N = sampledUnitSphere.Element + 3 * i;

                            NProx = RVLDOTPRODUCT3(pPt->N, N);

                            if (NProx > maxNProx)
                            {
                                maxNProx = NProx;

                                pModel->iCTIElement[iPt] = i;
                            }
                        }
                    }

                    delete[] PC.Element;
                }

                // fclose(fpPC);
            }

            iModel++;
        }
    }

    if (bCAG)
        CreateComponentAssociationGraph();

    if (bMST)
    {
        // Component Association.

        ComponentAssociationMST();

        // printf("Identify the components corresponding to the selected component...\n");

        // int nMCompsTotal = nMCompsTotalOU[0] + nMCompsTotalOU[1];

        // float *fLabel = new float[nMCompsTotal];

        // ComponentAssociation(fLabel);

        // FILE *fpfLabel = fopen((std::string(resultsFolder) + "\\flabel.dat").data(), "wb");

        // fwrite(fLabel, sizeof(float), nMCompsTotal, fpfLabel);

        // fclose(fpfLabel);

        // delete[] fLabel;

        // char *VNPartModelDataBaseFileName = RVLCreateFileName(modelDataBase, ".dat", -1, ".vn.part.dat");

        // FILE *fpVNDB = fopen(VNPartModelDataBaseFileName, "wb");

        // delete[] VNPartModelDataBaseFileName;

        // classes[0].Save(this, fpVNDB);

        // fclose(fpVNDB);
    }
}

void VNClassifier::LearnPrimitives(char *primitiveFileName)
{
    Eigen::MatrixXf A(3, 66);

    A = alignment.ConvexTemplatenT();

    // Load primitives from file and create convex hulls.

    FILE *fp = fopen(primitiveFileName, "r");

    if (fp == NULL)
    {
        printf("ERROR: Can't open %s\n", primitiveFileName);

        return;
    }

    printf("Loading primitives from %s...", primitiveFileName);

    Array<Mesh> convexHulls;

    fscanf(fp, "%d\n", &(convexHulls.n));

    if (convexHulls.n <= 0)
    {
        fclose(fp);

        return;
    }

    memConvexHull.Create(100000000);

    convexHullVertices.w = 3;

    convexHulls.Element = new Mesh[convexHulls.n];

    int iPrimitive, iPt, nPts;
    char tempString[100];
    float *P;
    Mesh *pConvexHull;

    for (iPrimitive = 0; iPrimitive < convexHulls.n; iPrimitive++)
    {
        fscanf(fp, "%d\t%s\t%d\t%d\n", &convexHulls.Element[iPrimitive].idPrimitive, tempString, &convexHulls.Element[iPrimitive].idCluster, &nPts);

        RVLCopyString(tempString, &convexHulls.Element[iPrimitive].name);

        convexHullVertices.Element = new float[3 * nPts];

        convexHullVertices.h = nPts;

        if (nPts > 0)
        {
            for (iPt = 0; iPt < nPts; iPt++)
            {
                P = convexHullVertices.Element + 3 * iPt;

                fscanf(fp, "%f\t%f\t%f\n", P, P + 1, P + 2);
            }
        }

        pConvexHull = convexHulls.Element + iPrimitive;

        pConvexHull->ConvexHull(convexHullVertices, &memConvexHull, false);

        delete[] convexHullVertices.Element;
    }

    fclose(fp);

    printf("completed.\n");

    int nCT = alignment.convexTemplate.n;

    int i, iCTI;
    PSGM_::ModelInstance *CTIMem;

    if (bGenerateModelCTIs)
    {
        // Generate primitive CTIs.

        FILE *fp = fopen(modelDataBase, "w");

        Array<int> iCTIArray;
        PSGM_::ModelInstance *pCTI;
        Array<Pose3D> referenceFrames;
        float *R;

        for (iPrimitive = 0; iPrimitive < convexHulls.n; iPrimitive++)
        {
            printf("%d\n", iPrimitive);

            pConvexHull = convexHulls.Element + iPrimitive;

            ReferenceFrames(pConvexHull, referenceFrames);

            CTIMem = new PSGM_::ModelInstance[referenceFrames.n];

            alignment.CTISet.pCTI.Element = new PSGM_::ModelInstance *[referenceFrames.n];

            iCTIArray.Element = new int[referenceFrames.n];

            for (iCTI = 0; iCTI < referenceFrames.n; iCTI++)
            {
                pCTI = CTIMem + iCTI;

                alignment.CTISet.pCTI.Element[iCTI] = pCTI;

                pCTI->iCluster = 0;
                pCTI->iModel = iPrimitive;

                R = referenceFrames.Element[iCTI].R;

                RVLCOPYMX3X3(R, pCTI->R);
                RVLNULL3VECTOR(pCTI->t);

                alignment.CTI(pConvexHull, pCTI);

                iCTIArray.Element[iCTI] = iCTI;
            }

            iCTIArray.n = alignment.CTISet.pCTI.n;

            // if (iPrimitive == 19)
            //	int debug = 0;

            iCTI = alignment.ReferenceFrame(pConvexHull, iCTIArray, alignment.CTISet.pCTI.Element);

            alignment.SaveCTI(fp, alignment.CTISet.pCTI.Element[iCTI], iPrimitive);

            delete[] alignment.CTISet.pCTI.Element;
            delete[] iCTIArray.Element;
            delete[] CTIMem;
            delete[] referenceFrames.Element;
        }

        fclose(fp);
    }
    else
    {
        // Create translational subspace.

        float alpha;

        float *MT = alignment.CreateTranslationalSubspace(alpha);

        // D <- primitive CTIs in the cannonical form.

        int nPrimitives = alignment.MCTISet.pCTI.n;

        float *d = new float[nCT];

        float *D = new float[nPrimitives * nCT];

        PSGM_::ModelInstanceElement *CTIDesc;
        float beta;
        float q[4];

        for (iPrimitive = 0; iPrimitive < nPrimitives; iPrimitive++)
        {
            CTIDesc = alignment.MCTISet.pCTI.Element[iPrimitive]->modelInstance.Element;

            for (i = 0; i < nCT; i++)
                d[i] = CTIDesc[i].d;

            alignment.CreateSubpace(d, MT, alpha, beta, q);

            memcpy(D + iPrimitive * nCT, MT + 3 * nCT, nCT * sizeof(float));
        }

        // Create alignment tree.

        MSTree MST;

        MST.Init();

        float **RArray = new float *[nPrimitives];
        float **tArray = new float *[nPrimitives];
        int **iRArray = new int *[nPrimitives];
        float **sArray = new float *[nPrimitives];

        int iPrimitive_;
        float *E;
        float *R;
        int *RP, *iD;
        int *iR;
        float *t;
        float *s;
        int iR_;
        float *d_, *d__;
        float cs, e;

        if (bCreateAlignmentTree)
        {
            // Align primitives.
            // alignment.LoadModelDataBase();

            for (iPrimitive = 0; iPrimitive < nPrimitives; iPrimitive++)
            {
                if (iPrimitive > 0)
                {
                    E = new float[iPrimitive];

                    R = new float[9 * iPrimitive];

                    t = new float[3 * iPrimitive];

                    iR = new int[iPrimitive];

                    s = new float[iPrimitive];

                    d__ = D + iPrimitive * nCT;

                    for (iPrimitive_ = 0; iPrimitive_ < iPrimitive; iPrimitive_++)
                    {
                        // if (iPrimitive == 990 && iPrimitive_ == 103)
                        //	int debug = 0;

                        // Approach 0:

                        // E[iPrimitive_] = -Align(convexHulls.Element[iPrimitive_].faces, convexHulls.Element[iPrimitive].faces, R + 9 * iPrimitive_);

                        // Approach 1:

                        // alignment.ObjectAlignment(alignment.MCTISet.SegmentCTIs.Element[iPrimitive],
                        // alignment.MCTISet.pCTI.Element,
                        // alignment.MCTISet.SegmentCTIs.Element[iPrimitive_],
                        // alignment.MCTISet.pCTI.Element, A, R + 9 * iPrimitive_, t + 3 * iPrimitive_, false, E + iPrimitive_);

                        // Approach 2:

                        if (iPrimitive >= 798 && iPrimitive_ >= 798)
                        {
                            E[iPrimitive_] = 0.0f;
                            iR[iPrimitive_] = 0;
                        }
                        else
                        {
                            d_ = D + iPrimitive_ * nCT;

                            for (iR_ = 0; iR_ < alignment.CTIDescMap.nCTI; iR_++)
                            {
                                iD = alignment.CTIDescMap.iD + nCT * iR_;

                                for (i = 0; i < nCT; i++)
                                    d[i] = d_[iD[i]];

                                RVLDOTPRODUCT(d, d__, nCT, cs, i);

                                e = 1.0f - cs * cs;

                                if (iR_ == 0 || e < E[iPrimitive_])
                                {
                                    E[iPrimitive_] = e;

                                    iR[iPrimitive_] = iR_;
                                }
                            }
                        }
                    }

                    RArray[iPrimitive] = R;
                    tArray[iPrimitive] = t;
                    iRArray[iPrimitive] = iR;
                    sArray[iPrimitive] = s;
                }
                else
                    E = NULL;

                MST.Update(E);
                printf("Adding primitive: %d\n", iPrimitive);
            }
        }
        else
        {
            // Load alignment tree.
        }

        // Align primitives.

        int *RAligned = new int[9 * nPrimitives];

        int iRefPrimitive = 0;

        MST.T.OrientTreeEdges(iRefPrimitive);

        int *queue = new int[nPrimitives];

        int *pPut = queue;

        *(pPut++) = iRefPrimitive;

        RP = RAligned + 9 * iRefPrimitive;

        RVLUNITMX3(RP);

        int *pFetch = queue;

        GRAPH::EdgePtr<GRAPH::Edge> *pEdgePtr;
        int *RP_, *RPP;

        while (pFetch < pPut)
        {
            iPrimitive = *(pFetch++);

            RP = RAligned + 9 * iPrimitive;

            pEdgePtr = MST.T.NodeArray.Element[iPrimitive].EdgeList.pFirst;

            while (pEdgePtr)
            {
                if (pEdgePtr->pEdge->iVertex[0] == iPrimitive)
                {
                    iPrimitive_ = pEdgePtr->pEdge->iVertex[1];

                    *(pPut++) = iPrimitive_;

                    RP_ = RAligned + 9 * iPrimitive_;

                    if (iPrimitive < iPrimitive_)
                    {
                        iR_ = iRArray[iPrimitive_][iPrimitive];

                        RPP = alignment.CTIDescMap.R + 9 * iR_;

                        RVLMXMUL3X3(RP, RPP, RP_);
                    }
                    else
                    {
                        iR_ = iRArray[iPrimitive][iPrimitive_];

                        RPP = alignment.CTIDescMap.R + 9 * iR_;

                        RVLMXMUL3X3T2(RP, RPP, RP_);
                    }
                }

                pEdgePtr = pEdgePtr->pNext;
            }
        }

        delete[] queue;

        // Create aligned CTIs.

        float *DAligned = new float[nCT * nPrimitives];

        int R_[9];

        for (iPrimitive = 0; iPrimitive < nPrimitives; iPrimitive++)
        {
            RP = RAligned + 9 * iPrimitive;

            d = D + nCT * iPrimitive;

            d_ = DAligned + nCT * iPrimitive;

            for (iR_ = 0; iR_ < alignment.CTIDescMap.nCTI; iR_++)
            {
                RP_ = alignment.CTIDescMap.R + 9 * iR_;

                for (i = 0; i < 9; i++)
                    if (RP[i] != RP_[i])
                        break;

                if (i >= 9)
                    break;
            }

            // if (iR_ >= alignment.CTIDescMap.nCTI)
            //	int debug = 0;

            iD = alignment.CTIDescMap.iD + nCT * iR_;

            for (i = 0; i < nCT; i++)
                d_[iD[i]] = d[i];
        }

        // Save aligned CTI database.

        uchar *bAllValid = new uchar[nCT];

        for (i = 0; i < nCT; i++)
            bAllValid[i] = true;

        FILE *fpAlignedCTIDB = fopen(primitiveDataBase, "w");

        float fR[9];

        RVLUNITMX3(fR);

        for (iPrimitive = 0; iPrimitive < nPrimitives; iPrimitive++)
        {
            RP = RAligned + 9 * iPrimitive;

            SaveDescriptor(fpAlignedCTIDB, DAligned + nCT * iPrimitive, bAllValid, fR, iPrimitive, 0);
        }

        fclose(fpAlignedCTIDB);

        // Visualization.

        float t_[3];

        RVLNULL3VECTOR(t_);

        // float R_[9], R__[9], t_[3], t__[3];
        int R__[9];
        int iPrimitive__;
        float s_;
        float M3x3Tmp[9], V3Tmp[3];
        if (bVisualization)
        {
            Visualizer visualizer;

            visualizer.Create();

            double color[3] = {1, 1, 1};

            bool bContinue = true;
            while (bContinue)
            {
                printf("Enter primitive1 ID: ");

                scanf("%d", &iRefPrimitive);

                if (iRefPrimitive < 0)
                    break;

                printf("Enter primitive2 ID: ");

                scanf("%d", &iPrimitive);

                if (iPrimitive < 0 || iPrimitive == iRefPrimitive)
                    break;

                RVLUNITMX3(R_);

                RVLCOPYMX3X3(R_, fR);

                alignment.DisplayCTI(&visualizer, sampledUnitSphere.Element, sampledUnitSphere.h, DAligned + iRefPrimitive * nCT, fR, t_, NULL, color);

                // RP = RAligned + 9 * iRefPrimitive;

                // RP_ = RAligned + 9 * iPrimitive;

                // RVLMXMUL3X3T1(RP, RP_, R_);

                // MST.T.OrientTreeEdges(iRefPrimitive);

                // iPrimitive_ = iPrimitive;

                // while (iPrimitive_ != iRefPrimitive)
                //{
                //	pEdgePtr = MST.T.NodeArray.Element[iPrimitive_].EdgeList.pFirst;

                //	while (pEdgePtr)
                //	{
                //		if (pEdgePtr->pEdge->iVertex[0] != iPrimitive_)
                //			break;

                //		pEdgePtr = pEdgePtr->pNext;
                //	}

                //	iPrimitive__ = pEdgePtr->pEdge->iVertex[0];

                //	if (iPrimitive__ < iPrimitive_)
                //	{
                //		iR_ = iRArray[iPrimitive_][iPrimitive__];

                //		RP = alignment.CTIDescMap.R + 9 * iR_;

                //		RVLMXMUL3X3(RP, R_, R__);

                //		//R = RArray[iPrimitive_] + 9 * iPrimitive__;
                //		//t = tArray[iPrimitive_] + 3 * iPrimitive__;

                //		//RVLMXMUL3X3T1(R, R, M3x3Tmp);

                //		//s = (M3x3Tmp[0] + M3x3Tmp[4] + M3x3Tmp[8]) / 3.0f;

                //		//RVLINVTRANSF3D(R, t, R__, t__);

                //		//RVLSCALEMX3X32(R__, s, M3x3Tmp);
                //		//RVLSCALE3VECTOR2(t__, s, V3Tmp);

                //		//RVLCOMPTRANSF3D(M3x3Tmp, V3Tmp, R_, t_, R__, t__);
                //	}
                //	else
                //	{
                //		iR_ = iRArray[iPrimitive__][iPrimitive_];

                //		RP = alignment.CTIDescMap.R + 9 * iR_;

                //		RVLMXMUL3X3T1(RP, R_, R__);

                //		//R = RArray[iPrimitive__] + 9 * iPrimitive_;
                //		//t = tArray[iPrimitive__] + 3 * iPrimitive_;

                //		//RVLCOMPTRANSF3D(R, t, R_, t_, R__, t__);
                //	}

                //	RVLCOPYMX3X3(R__, R_);

                //	iPrimitive_ = iPrimitive__;
                //}

                // int iCTI;

                // int iCTI = alignment.ObjectAlignment(convexHulls.Element + iPrimitive, alignment.MCTISet.SegmentCTIs.Element[iPrimitive],
                //	alignment.MCTISet.pCTI.Element);

                // alignment.OptimalTranslationAndScale(alignment.MCTISet.pCTI.Element[alignment.MCTISet.SegmentCTIs.Element[0].Element[0]],
                //	alignment.MCTISet.pCTI.Element[alignment.MCTISet.SegmentCTIs.Element[iPrimitive].Element[iCTI]], M, alpha, t_, s_);

                // float *RBox = alignment.MCTISet.pCTI.Element[alignment.MCTISet.SegmentCTIs.Element[iPrimitive].Element[iCTI]]->R;

                // RVLSCALEMX3X3(RBox, s_, R__);

                // RVLTRASPOSE3X3(R__, R_)

                RVLCOPYMX3X3(R_, fR);

                alignment.DisplayCTI(&visualizer, sampledUnitSphere.Element, sampledUnitSphere.h, DAligned + iPrimitive * nCT, fR, t_);

                visualizer.Run();

                visualizer.renderer->RemoveAllViewProps();
            }
        }

        //

        delete[] RAligned;
        delete[] DAligned;

        // Save edges and costs to file.

        FILE *fpEdges = fopen((std::string(resultsFolder) + "\\VNEdges.txt").data(), "w");
        FILE *fpRot = fopen((std::string(resultsFolder) + "\\VNRot.txt").data(), "w");
        FILE *fpCost = fopen((std::string(resultsFolder) + "\\VNCost.txt").data(), "w");
        RVLUNITMX3(R_);
        int temp;
        for (iPrimitive = 0; iPrimitive < nPrimitives; iPrimitive++)
        {
            for (iPrimitive_ = 0; iPrimitive_ < iPrimitive; iPrimitive_++)
            {
                fprintf(fpCost, "%f\n", MST.E[iPrimitive][iPrimitive_]);
            }

            pEdgePtr = MST.T.NodeArray.Element[iPrimitive].EdgeList.pFirst;
            while (pEdgePtr)
            {
                if (pEdgePtr->pEdge->iVertex[0] == iPrimitive)
                {
                    iPrimitive__ = pEdgePtr->pEdge->iVertex[0];
                    iPrimitive_ = pEdgePtr->pEdge->iVertex[1];

                    if (iPrimitive__ < iPrimitive_)
                    {
                        temp = iPrimitive__;
                        iPrimitive__ = iPrimitive_;
                        iPrimitive_ = temp;
                    }

                    R = RArray[iPrimitive__] + 9 * iPrimitive_;

                    // RVLMXMUL3X3(R, R_, R__);

                    fprintf(fpRot, "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8]);

                    fprintf(fpEdges, "%d %d\n", iPrimitive__, iPrimitive_);
                }

                pEdgePtr = pEdgePtr->pNext;
            }
        }
        fclose(fpEdges);
        fclose(fpCost);
        fclose(fpRot);

        // Read edges and costs from file:

        /*FILE *fpEdges = fopen((std::string(resultsFolder) + "\\VNEdges.txt").data(), "r");
        FILE *fpCost = fopen((std::string(resultsFolder) + "\\VNCost.txt").data(), "r");
        MST.Init();
        for (iPrimitive = 0; iPrimitive < convexHulls.n; iPrimitive++)
        {
        if (iPrimitive > 0)
        {
        E = new float[iPrimitive];

        R = new float[9 * iPrimitive];

        for (iPrimitive_ = 0; iPrimitive_ < iPrimitive; iPrimitive_++)
        fscanf(fpCost, "%f\t", &E[iPrimitive_]);

        }
        else
        E = NULL;

        MST.Update(E);
        }

        int iRefPrimitive = 0;

        MST.T.OrientTreeEdges(iRefPrimitive);
        */

        // Free memory.

        for (iPrimitive = 1; iPrimitive < nPrimitives; iPrimitive++)
        {
            delete[] RArray[iPrimitive];
            delete[] tArray[iPrimitive];
            delete[] iRArray[iPrimitive];
            delete[] sArray[iPrimitive];
        }
        delete[] RArray;
        delete[] tArray;
        delete[] iRArray;
        delete[] sArray;
        delete[] MT;
        delete[] D;
        delete[] d;
    } // if (!bGenerateModelCTIs)

    // Free memory.

    delete[] convexHulls.Element;
}

void VNClassifier::Interpret(Mesh *pMesh)
{
    if (problem == RVLRECOGNITION_PROBLEM_CLASSIFICATION)
    {
        if (bPlusPlus)
            Interpret2(pMesh);
        else
            Interpret1(pMesh);
    }
    else if (problem == RVLRECOGNITION_PROBLEM_SHAPE_INSTANCE_DETECTION)
        Interpret3(pMesh);
}

void VNClassifier::Interpret1(Mesh *pMesh)
{
#ifdef RVLVN_TIME_MESUREMENT
    LARGE_INTEGER CNTRSegmentationSTART, CNTRSegmentationEND;
    LARGE_INTEGER CNTRDescriptorGenerationSTART, CNTRDescriptorGenerationEND;

    LARGE_INTEGER frequency;
    QueryPerformanceFrequency((LARGE_INTEGER *)&frequency);
#endif

#ifdef RVLVN_TIME_MESUREMENT
    // Timer START
    QueryPerformanceCounter((LARGE_INTEGER *)&CNTRSegmentationSTART);
#endif
    // Determine the greatest number of latent variables per class.

    int maxnLatentVariables = 0;

    ClassData *pVNClass;
    int iClass;

    for (iClass = 0; iClass < classArray.n; iClass++)
    {
        pVNClass = classArray.Element + iClass;

        if (pVNClass->M.Element == NULL)
            continue;

        if (pVNClass->M.h > maxnLatentVariables)
            maxnLatentVariables = pVNClass->M.h;
    }

    if (maxnLatentVariables == 0)
        return;

    // Detect ground plane.

    bGnd = false;

    Array<int> groundPlaneSurfelArray;

    groundPlaneSurfelArray.Element = NULL;
    groundPlaneSurfelArray.n = 0;

    if (!bDetectSupportingSurfaces && b3DNetTestSet)
    {
        _3DNetGround(alignment.sceneFileName, NGnd, dGnd);

        bGnd = true;

        pSurfelDetector->bGnd = true;
        RVLCOPY3VECTOR(NGnd, pSurfelDetector->NGnd);
        pSurfelDetector->dGnd = dGnd;
    }

    // Detect surfels.

    pSurfels->Init(pMesh);

    pSurfelDetector->Init(pMesh, pSurfels, pMem);

    printf("Segmentation to surfels...");

    pSurfelDetector->Segment(pMesh, pSurfels);

    printf("completed.\n");

    int nSurfels = pSurfels->NodeArray.n;

    printf("No. of surfels = %d\n", nSurfels);

    // Relations between adjacent surfels.

#ifdef RVLSURFEL_IMAGE_ADJACENCY
    pSurfels->SurfelRelations(pMesh);
#endif

    // Detect ground plane.

    if (pSurfels->bGroundContactVertices && !bGnd && !bDetectSupportingSurfaces)
    {
        groundPlaneSurfelArray.Element = new int[pSurfels->NodeArray.n];

        pSurfels->DetectDominantPlane(pMesh, groundPlaneSurfelArray, NGnd, dGnd);

        bGnd = true;
    }

    // Detect vertices.

    printf("Detect vertices.\n");

    pSurfels->DetectVertices(pMesh);

    // Detect supporting surfaces and the ground plane.

    if (bDetectSupportingSurfaces)
    {
        DetectSupportingSurfaces(pMesh);

        bGnd = true;
    }

    /// Detect objects.

    float RSG[9];
    float tSG[3];
    float rVOI;

    if (b3DNetVOI)
        PSGM_::_3DNetVOI(RSG, tSG, rVOI);

    int iObject;

    if (bBoundingBoxes)
    {
        // Load bounding box.

        char *boundingBoxFileName = RVLCreateFileName(sceneFileName, ".ply", -1, ".bbx");

        FILE *fpBoundingBox = fopen(boundingBoxFileName, "r");

        delete[] boundingBoxFileName;

        Rect<int> bbox;
        int bboxw, bboxh;

        fscanf(fpBoundingBox, "%d\t%d\t%d\t%d\n", &(bbox.minx), &(bbox.miny), &bboxw, &bboxh);

        bbox.maxx = bbox.minx + bboxw;
        bbox.maxy = bbox.miny + bboxh;

        // Create foreground object.

        pObjects->NodeArray.n = 1;

        RVL_DELETE_ARRAY(pObjects->NodeArray.Element);

        pObjects->NodeArray.Element = new GRAPH::AggregateNode<SURFEL::AgEdge>;

        GRAPH::AggregateNode<SURFEL::AgEdge> *pObjectGraphNode = pObjects->NodeArray.Element;

        RVL_DELETE_ARRAY(pObjects->elementMem);

        pObjects->elementMem = new QLIST::Index[pSurfels->NodeArray.n];

        QList<QLIST::Index> *pSurfelList = &(pObjectGraphNode->elementList);

        RVLQLIST_INIT(pSurfelList);

        QLIST::Index *pSurfelIdx = pObjects->elementMem;

        pObjectGraphNode->size = 0;

        float groundFilterThr = 0.01f;

        VN_::SurfelMask sceneSurfelMask;

        CreateSurfelMask(pMesh, sceneSurfelMask);

        Surfel *pSurfel;
        float eGnd;
        QLIST::Index2 *pPtIdx;
        int iPix, u, v, intersection;

        for (int iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++)
        {
            pSurfel = pSurfels->NodeArray.Element + iSurfel;

            if (pSurfel->size == 0)
                continue;

            if (!(pSurfel->flags & RVLSURFEL_FLAG_GND))
            {
                eGnd = RVLDOTPRODUCT3(pSurfel->P, NGnd) - dGnd;

                if (eGnd > groundFilterThr)
                {
                    intersection = 0;

                    pPtIdx = pSurfel->PtList.pFirst;

                    while (pPtIdx)
                    {
                        iPix = pPtIdx->Idx;
                        u = iPix % camera.w;
                        v = iPix / camera.w;

                        if (IsInRect<int>(u, v, bbox))
                            intersection++;

                        pPtIdx = pPtIdx->pNext;
                    }

                    if (100 * intersection / pSurfel->size >= 50)
                    {
                        sceneSurfelMask.mask[iSurfel] = true;

                        RVLQLIST_ADD_ENTRY(pSurfelList, pSurfelIdx);

                        pSurfelIdx->Idx = iSurfel;

                        pSurfelIdx++;

                        pObjectGraphNode->size += pSurfel->size;
                    }
                }
            }
        }

        DisplaySurfelMask(sceneSurfelMask, &bbox);

        pObjects->nValidObjects = -1;
        pObjects->sortedObjectArray.n = -1;

        pObjects->GetVertices();

        delete[] sceneSurfelMask.mem;

        iObject = 0;
    }
#ifdef RVLSURFEL_IMAGE_ADJACENCY
    else if (b3DNetTestSet)
    {
        // Detect objects as connected surfel sets.

        pObjects->pMesh = pMesh;

        pObjects->CreateObjectsAsConnectedComponents(groundPlaneSurfelArray, connectedComponentMaxDist, connectedComponentMinSize);

        RVL_DELETE_ARRAY(groundPlaneSurfelArray.Element);

        // Sort objects.

        pObjects->nValidObjects = -1;
        pObjects->sortedObjectArray.n = -1;

        pObjects->SortObjects();

        // Assign vertices to objects.

        pObjects->GetVertices();

        // Detect objects in VOI

        uchar mask = RVLPCSEGMENT_OBJECT_FLAG_GND;
        uchar flags = 0x00;

        if (pObjects->b3DNetVOI)
        {
            pObjects->ObjectsInVOI(RSG, tSG, rVOI);

            mask |= RVLPCSEGMENT_OBJECT_FLAG_IN_VOI;
            flags |= RVLPCSEGMENT_OBJECT_FLAG_IN_VOI;
        }

        RECOG::GroupObjectsAccordingToCTIProximity(pMesh, pObjects, &alignment, 0.100f, flags, flags);

        // Get foreground object.

        iObject = pObjects->GetForegroundObject();

        if (iObject < 0)
            return;
    } // if (b3DNetTestSet)
#endif
    else
    {
        // Identify surfels in VOI.

        if (b3DNetVOI)
        {
            pSurfels->SurfelsInVOI(RSG, tSG, rVOI);

            convexClustering.clusteringSurfelFlags = convexClustering.clusteringSurfelFlagMask = RVLSURFEL_FLAG_VOI;
            concaveClustering.clusteringSurfelFlags = concaveClustering.clusteringSurfelFlagMask = RVLSURFEL_FLAG_VOI;
        }

        // Cluster surfels into convex and concave surfaces.

        RVL_DELETE_ARRAY(SClusters.Element);

        RVL_DELETE_ARRAY(clusterMap);

        clusterMap = new int[pSurfels->NodeArray.n];

        int nCClusters, nUClusters;

        PSGM_::ConvexAndConcaveClusters(pMesh, pSurfels, pSurfelDetector, &convexClustering, &concaveClustering,
                                        SClusters, nCClusters, nUClusters, 0, clusterMap, -1, -1, true, 0,
                                        visualizationData.bVisualizeConvexClusters, visualizationData.bVisualizeConcaveClusters, visualizationData.bVisualizeSurfels);

        // Compute descriptors.

        int descriptorBlockSize = SO3Samples.c * sampledUnitSphere.h;

        int descriptorMemSize = descriptorBlockSize * (SClusters.n + nUClusters);

        float *d = new float[descriptorMemSize];

        float *d_ = d;

        uchar *bd = new uchar[descriptorMemSize];

        uchar *bd_ = bd;

        int iCluster;
        SceneCluster *pCluster;
        PSGM_::Cluster *pCluster_;
        float *dU_;
        uchar *bdU_;

        for (iCluster = 0; iCluster < SClusters.n; iCluster++)
        {
            pCluster = SClusters.Element + iCluster;

            pCluster_ = (PSGM_::Cluster *)(pCluster->vpCluster);

            if (pCluster->type == RVLVN_CLUSTER_TYPE_CONVEX)
            {
                ComputeDescriptors(pCluster_->iSurfelArray, pCluster_->iVertexArray, d_, bd_);

                pCluster_->d = d_;
                pCluster_->bd = bd_;
                pCluster_->dU = NULL;
                pCluster_->bdU = NULL;

                d_ += descriptorBlockSize;

                bd_ += descriptorBlockSize;
            }
            else if (pCluster->type == RVLVN_CLUSTER_TYPE_CONCAVE)
            {
                dU_ = d_ + descriptorBlockSize;

                bdU_ = bd_ + descriptorBlockSize;

                ComputeDescriptors(pCluster_->iSurfelArray, pCluster_->iVertexArray, d_, bd_, dU_, bdU_);

                pCluster_->d = d_;
                pCluster_->bd = bd_;
                pCluster_->dU = dU_;
                pCluster_->bdU = bdU_;

                d_ = dU_ + descriptorBlockSize;

                bd_ = bdU_ + descriptorBlockSize;
            }
        }

        //

        RVLMEM_ALLOC_STRUCT_ARRAY(pMem, float, descriptorBlockSize, d_);
        RVLMEM_ALLOC_STRUCT_ARRAY(pMem, uchar, descriptorBlockSize, bd_);

        PSGM_::Cluster *pCluster__[2];

        pCluster = SClusters.Element + 2;

        pCluster__[0] = (PSGM_::Cluster *)(pCluster->vpCluster);

        pCluster = SClusters.Element + 13;

        pCluster__[1] = (PSGM_::Cluster *)(pCluster->vpCluster);

        MergeDescriptors(pCluster__[0]->d, pCluster__[0]->bd, pCluster__[1]->d, pCluster__[1]->bd, 1.0f, d_, bd_);

        dU_ = pCluster__[1]->dU;
        bdU_ = pCluster__[1]->bdU;

#ifdef RVLVN_RECORD_RESULTS

        FILE *fp = fopen((std::string(resultsFolder) + "\\scenedescriptors.txt").data(), "w");

        SaveDescriptors(fp, pCluster__[0]->d, pCluster__[0]->bd, 2, RVLVN_CLUSTER_TYPE_CONVEX);

        SaveDescriptors(fp, pCluster__[1]->dU, pCluster__[1]->bdU, 13, RVLVN_CLUSTER_TYPE_CONCAVE);

        SaveDescriptors(fp, d_, bd_, SClusters.n, RVLVN_CLUSTER_TYPE_CONVEX);

        fclose(fp);
#endif
        //

        ClassData *pClass = classArray.Element + 3;

        float *qOpt = new float[pClass->M.h];

        int iR;

        float E = ProjectToLatentSubspace(d_, bd_, dU_, bdU_, pClass, qOpt, iR);

        delete[] d;
        delete[] bd;

        float *R_ = SO3Samples.Element + 9 * iR;

        Array<int> iSurfelArray;

        iSurfelArray.Element = new int[pSurfels->NodeArray.n];

        iSurfelArray.n = 0;

        Array<int> iVertexArray;

        iVertexArray.Element = new int[pSurfels->vertexArray.n];

        iVertexArray.n = 0;

        int i, j;

        for (i = 0; i < 2; i++)
        {
            for (j = 0; j < pCluster__[i]->iVertexArray.n; j++)
                iVertexArray.Element[iVertexArray.n++] = pCluster__[i]->iVertexArray.Element[j];

            for (j = 0; j < pCluster__[i]->iSurfelArray.n; j++)
                iSurfelArray.Element[iSurfelArray.n++] = pCluster__[i]->iSurfelArray.Element[j];
        }

        Camera camera;

        camera.fu = 525;
        camera.fv = 525;
        camera.uc = 320;
        camera.vc = 240;
        camera.w = pMesh->width;
        camera.h = pMesh->height;

        SURFEL::SceneSamples sceneSamples;

        pSurfels->SampleSurfelSet(pMesh, iSurfelArray, iVertexArray, camera, sceneSamples);

        Array<OrientedPoint> PtArray;

        PtArray.n = sceneSamples.imagePtArray.h;

        PtArray.Element = new OrientedPoint[PtArray.n];

        float *c;

        // memset(qOpt + 4, 0, 6 * sizeof(float));

        float R[9];
        float EHull;

        bool bGnd = fitParams.bGnd;

        fitParams.bInit = false;
        fitParams.bGnd = fitParams.bGnd2;

        VN *pModel = models[pClass->iMetaModel];

        pModel->FitRotLMCC(pSurfels, iVertexArray, sceneSamples, pClass, NULL, NULL, R_, fitParams, qOpt, R, EHull, false, &camera);

        float *dM = new float[pClass->M.w];

        fitParams.bInit = true;
        fitParams.bGnd = bGnd;

        RVLMULMXTVECT(pClass->M.Element, qOpt, pClass->M.h, pClass->M.w, dM, i, j, c);

        float t[3];

        RVLNULL3VECTOR(t);

        pModel->Project(dM, R, t, camera, sceneSamples.imagePtArray, PtArray);

        // Visualize all clusters.

        if (visualizationData.bVisualizeAllClusters)
        {
            uchar SelectionColor[] = {0, 255, 0};

            RandomColors(SelectionColor, visualizationData.clusterColor, SClusters.n);

            InitDisplay(visualizationData.pVisualizer, pMesh, SelectionColor);

            pSurfels->DisplayEdgeFeatures();

            DisplayClusters();

            pSurfels->DisplayVertices();

            unsigned char color[] = {0, 128, 255};

            Array<Point> PtArray_;

            PtArray_.Element = new Point[PtArray.n];

            PtArray_.n = 0;

            int iPt;
            float *P_;
            OrientedPoint *pP;

            for (iPt = 0; iPt < PtArray.n; iPt++)
            {
                pP = PtArray.Element + iPt;

                if (pP->P[2] <= 2.0f)
                {
                    P_ = PtArray_.Element[PtArray_.n++].P;

                    RVLCOPY3VECTOR(pP->P, P_);
                }
            }

            visualizationData.pVisualizer->DisplayPointSet<float, Point>(PtArray_, color, 6.0f);

            visualizationData.pVisualizer->Run();

            // visualizationData.pVisualizer->renderer->RemoveAllViewProps();

            delete[] visualizationData.clusterColor;
            delete[] PtArray_.Element;
        }

        delete[] qOpt;
        delete[] iVertexArray.Element;
        delete[] PtArray.Element;
        delete[] dM;
    } // if (!b3DNetTestSet)

    // Create CTIs.

    alignment.CTISet.Init();

    SURFEL::Object *pObject = pObjects->objectArray.Element + iObject;

    // RVLCOPY3VECTOR(NGnd, alignment.NGnd);
    // alignment.dGnd = dGnd;
    // alignment.bGnd = true;

    // alignment.CTIs(pObject->surfelList, pObject->iVertexArray, 0, iObject, &(alignment.CTISet), pMem, true);

    // alignment.CTISet.CopyCTIsToArray();

    //// Save model instances to a file.

    // printf("Save model instances to a file.\n");

    // char *PSGModelInstanceFileName = RVLCreateString(alignment.sceneFileName);

    // sprintf(PSGModelInstanceFileName + strlen(PSGModelInstanceFileName) - 3, "cti");

    // FILE *fp = fopen(PSGModelInstanceFileName, "w");

    // delete[] PSGModelInstanceFileName;

    // alignment.SaveModelInstances(fp); //Vidovic

    // fclose(fp);

    //// Align the scene object with a model.

    // alignment.pMesh = pMesh;

    // alignment.CreateClusterFromObject(pObjects, iObject);

    // int iModel;
    float R[9], t[3];

    // alignment.Classify(pMesh, pVNClass->iRefInstance, pVNClass->iRefInstance + 1, iModel, R, t);

    // FILE *fpR = fopen("R.txt", "w");

    // PrintMatrix<float>(fpR, R, 3, 3);

    // fclose(fpR);

    ///

    // float s;
    // float R_[9];
    // float t_[3];

    // RVLEXTRACTSCALEFROMROT(R, s, R_);

    // float sSM = 1.0f / s;

    // RVLSCALE3VECTOR(t, sSM, t_);

    // float RSM[9];
    // float tSM[3];

    // RVLINVTRANSF3D(R_, t, RSM, tSM);

    // Array<TangentVertexCorrespondence> correspondences;

    // correspondences.Element = new TangentVertexCorrespondence[alignment.convexTemplate.n];

    // float score;
    // float RSM_[9];
    // float tSM_[3];

    // TangentAlignment(pSurfels, pObject->iVertexArray, sSM, alignment.MTGSet.A, refModel.d, R_, t_, 50.0f * sSM,
    //	score, correspondences, R_, t_);

    // RVLINVTRANSF3D(RSM_, tSM_, R_, t);
    // RVLSCALE3VECTOR(t, s, t);
    // RVLSCALE3VECTOR(t_, s, t);
    // RVLSCALEMX3X3(R_, s, R);

    // float *q0 = new float[pVNClass->M.h];

    // RVLCOPY3VECTOR(t, q0);

    // q0[3] = sqrt(RVLDOTPRODUCT3(R, R));

    // memset(q0 + 4, 0, (pVNClass->M.h - 4) * sizeof(float));

    Camera camera;

    camera.fu = 525;
    camera.fv = 525;
    camera.uc = 320;
    camera.vc = 240;
    camera.w = pMesh->width;
    camera.h = pMesh->height;

    Array<int> iSurfelArray;

    iSurfelArray.Element = new int[pSurfels->NodeArray.n];

    QLIST::CopyToArray(&(pObject->surfelList), &iSurfelArray);

    SURFEL::SceneSamples sceneSamples;

    pSurfels->SampleSurfelSet(pMesh, iSurfelArray, pObject->iVertexArray, camera, sceneSamples);

    delete[] iSurfelArray.Element;

    if (bGndConstraint)
    {
        sceneSamples.PGnd = new float[3 * pObject->iVertexArray.n];

        pSurfels->ProjectVerticesOntoGroundPlane(pObject->iVertexArray, NGnd, dGnd, sceneSamples.PGnd);
    }
    else
        sceneSamples.PGnd = NULL;

    // Rect<float> ROI;

    // pSurfels->GetDepthImageROI(pObject->iVertexArray, camera, ROI);

    // Array2D<float> imagePtArray;

    // SampleRect<float>(&ROI, 10.0f, 32, imagePtArray);

    fitParams.bInit = true;

    RVLNULL3VECTOR(t);

    float *q = new float[maxnLatentVariables];

    int iMostProbableClass = -1;

    float *qOpt = new float[maxnLatentVariables];

    bool bFirst = true;

    float bestScore = 0.0f;

    FILE *fpInstances = fopen((std::string(resultsFolder) + "\\instances.txt").data(), "w");

    int iFirstClass, iLastClass;

    if (bSingleClass)
        iFirstClass = iLastClass = iSelectedClass;
    else
    {
        iFirstClass = 0;
        iLastClass = classArray.n - 1;
    }

    std::string resultsFolderName = std::string(alignment.resultsFolder);
    FILE *fpScore = fopen((resultsFolderName + "\\VNScore.txt").data(), "a");

    fitParams.fpScore = fpScore;

    int iMetaModel;
    VN *pModel;
    float score;
    float ROpt[9];

    for (iClass = iFirstClass; iClass <= iLastClass; iClass++)
    {
        pVNClass = classArray.Element + iClass;

        if (pVNClass->M.Element == NULL)
            continue;

#ifdef RVLVN_RECORD_RESULTS

        printf("Class %d\n", iClass);
#endif
        iMetaModel = pVNClass->iMetaModel;

        pModel = models[iMetaModel];

        // pModel->Transform(refModel.d, R_, t, d);

        // pModel->FitLM(pMesh, pSurfels, pObject->surfelList, pObject->iVertexArray, camera, pVNClass->M, R_, d, q, true);
        // pModel->FitRotLM(pMesh, pSurfels, pObject->surfelList, pObject->iVertexArray, camera, pVNClass, d, R_, fitParams, q, R__, true);
        score = pModel->FitRotLM(this, pMesh, pSurfels, pObject->iVertexArray, camera, sceneSamples, pVNClass, NULL,
                                 fitParams, q, R, bVisualization, bVisualizeBestInstance);

        if (bFirst || score > bestScore)
        {
            bestScore = score;

            RVLCOPYMX3X3(R, ROpt);

            memcpy(qOpt, q, pVNClass->M.h * sizeof(float));

            iMostProbableClass = iClass;

            bFirst = false;
        }
#ifdef RVLVN_RECORD_RESULTS

        printf("\n");
#endif
        // float *d = new float[pVNClass->M.w];

        // int i, j;
        // float *a;

        // RVLMULMXTVECT(pVNClass->M.Element, q, pVNClass->M.h, pVNClass->M.w, d, i, j, a);

        SaveLatentVector(fpInstances, q, iClass);

        // delete[] d;
    }

    fclose(fpInstances);

    fprintf(fpScore, "\n");

    fclose(fpScore);

#ifdef RVLVN_RECORD_RESULTS

    // printf("Most probable class: %d\n\n\n", iMostProbableClass);
#endif
    hypClass[iMostProbableClass]++;

#ifdef RVLVN_RECORD_RESULTS

    // confusion matrix:
    printf("confusion matrix\n");
    for (int rb = 0; rb < 10; rb++)
    {
        printf("\n%d: %d", rb, hypClass[rb]);
    }

    FILE *fpHyp = fopen((resultsFolderName + "\\VNClassify.txt").data(), "a");
    int command = 0;

    if (iMostProbableClass != iSelectedClass)
    {
        fprintf(fpHyp, "%s\n", alignment.sceneFileName);
    }
    fclose(fpHyp);
#endif
    Array<OrientedPoint> PtArray;

    PtArray.n = sceneSamples.imagePtArray.h;

    PtArray.Element = new OrientedPoint[PtArray.n];

    pVNClass = classArray.Element + iMostProbableClass;

    pModel = models[pVNClass->iMetaModel];

    float *d = new float[pModel->featureArray.n];

    int i, j;
    float *a;

    RVLMULMXTVECT(pVNClass->M.Element, qOpt, pVNClass->M.h, pVNClass->M.w, d, i, j, a);

    pModel->Project(d, ROpt, t, camera, sceneSamples.imagePtArray, PtArray);
    // pModel->Project(d, R, t, camera, imagePtArray, PtArray);
    // pModel->Project(refModel.d, R, t, camera, imagePtArray, PtArray);

    SURFEL::DeleteSceneSamples(sceneSamples);

    // FILE *fp = fopen("P.txt", "w");

    // PrintMatrix<float>(fp, PtArray.Element, PtArray.h, PtArray.w);

    // fclose(fp);

    // Visualization

    if (bVisualization)
    {
        unsigned char SelectionColor[] = {0, 255, 0};

        pSurfels->NodeColors(SelectionColor);

        Visualizer visualizer;

        visualizer.Create();

        // pObjects->InitDisplay(&visualizer, pMesh, SelectionColor);
        // pObjects->Display();

        alignment.InitDisplay(&visualizer, pMesh, SelectionColor);
        alignment.Display();

        unsigned char color[] = {0, 128, 255};

        Array<Point> PtArray_;

        MESH::CreatePointArrayFromOrientedPointArray(PtArray, PtArray_, 2.0f);

        visualizer.DisplayPointSet<float, Point>(PtArray_, color, 6.0f);

#ifdef NEVER
        Gripper grip;

        vtkSmartPointer<vtkTransform> TIn = vtkSmartPointer<vtkTransform>::New();

        double TGS[16], RGS[9], RGM[9];

        int n1, n2, n3, n4, n5;
        n1 = pVNClass->VNGraspNormals_.Element[0].n1;
        n2 = pVNClass->VNGraspNormals_.Element[0].n2;
        n3 = pVNClass->VNGraspNormals_.Element[0].n3;
        n4 = pVNClass->VNGraspNormals_.Element[0].n4;
        n5 = pVNClass->VNGraspNormals_.Element[0].n5;

        //	//mug:
        //	n1 = 11;
        //	n2 = 92;
        //	n3 = 15;
        //	n4 = 18;
        //	n5 = 0;

        //	//hammer:
        //	n1 = 99;
        //	n2 = 85;
        //	n3 = 89;
        //	n4 = 26;
        //	n5 = 92;
        //
        //	//banana:
        ///*	n1 = 57;
        //	n2 = 73;
        //	n3 = 55;
        //	n4 = 54;
        //	n5 = 0;*/
        //	n1 = 55;
        //	n2 = 72;
        //	n3 = 50;
        //	n4 = 57;
        //	n5 = 0;

        //	//bottle:
        //	//parallel planes for gripping
        //	n1 = 87;
        //	n2 = 72;
        //	//adjacent planes
        //	n3 = 89;
        //	n4 = 91;
        //	//top plane
        //	n5 = 61;

        //	//car:
        //	n1 = 55;
        //	n2 = 22;
        //	n3 = 59;
        //	n4 = 62;
        //	n5 = 44;

        //	n1 = 55;
        //	n2 = 22;
        //	n3 = 58;
        //	n4 = 61;
        //	n5 = 0;

        float width = abs(d[n1] + d[n2]);
        float tx[3], tz[3], ty[3], ts1[3], ts2[3];
        float a;
        ts1[0] = pModel->featureArray.Element[n3].N[0];
        ts1[1] = pModel->featureArray.Element[n3].N[1];
        ts1[2] = pModel->featureArray.Element[n3].N[2];

        ts2[0] = pModel->featureArray.Element[n4].N[0];
        ts2[1] = pModel->featureArray.Element[n4].N[1];
        ts2[2] = pModel->featureArray.Element[n4].N[2];

        RGM[1] = ty[0] = pModel->featureArray.Element[n1].N[0];
        RGM[4] = ty[1] = pModel->featureArray.Element[n1].N[1];
        RGM[7] = ty[2] = pModel->featureArray.Element[n1].N[2];

        RGM[2] = tz[0] = pModel->featureArray.Element[n5].N[0];
        RGM[5] = tz[1] = pModel->featureArray.Element[n5].N[1];
        RGM[8] = tz[2] = pModel->featureArray.Element[n5].N[2];

        RVLCROSSPRODUCT3(ty, tz, tx);

        RGM[0] = tx[0];
        RGM[3] = tx[1];
        RGM[6] = tx[2];

        RVLMXMUL3X3(ROpt, RGM, RGS);

        a = (d[n3] * RVLDOTPRODUCT3(ts2, ty) - d[n4] * RVLDOTPRODUCT3(ts1, ty)) / (RVLDOTPRODUCT3(ts1, tx) * RVLDOTPRODUCT3(ts2, ty) - RVLDOTPRODUCT3(ts2, tx) * RVLDOTPRODUCT3(ts1, ty));

        float b = d[n1] - 0.5 * width;

        float t[3] = {a * RGS[0] + b * RGS[1] + d[n5] * RGS[2], a * RGS[3] + b * RGS[4] + d[n5] * RGS[5], a * RGS[6] + b * RGS[7] + d[n5] * RGS[8]};

        RVLHTRANSFMX(RGS, t, TGS);

        TIn->SetMatrix(TGS);

        // float translate[3] = { d[11] + (width / 2), 0, 0 };
        grip.CreateGripper(width, TIn, t, visualizer.renderer);
#endif

        // visualizer.window->AddRenderer(grip.renderer);
        // visualizer.window->AddRenderer(visualizer.renderer);
        visualizer.Run();

        delete[] PtArray_.Element;
    }

#ifdef RVLVN_TIME_MESUREMENT
    // Timer END
    QueryPerformanceCounter((LARGE_INTEGER *)&CNTRSegmentationEND);
    timeInterpret += (CNTRSegmentationEND.QuadPart - CNTRSegmentationSTART.QuadPart) * 1000.0 / frequency.QuadPart;
    printf("time: %.5f", timeInterpret);
#endif
    delete[] d;
    delete[] q;
    delete[] qOpt;
    // delete[] q0;
    // delete[] correspondences.Element;
}

void VNClassifier::Interpret2(Mesh *pMesh)
{
    Primitives(pMesh);
}

void VNClassifier::Interpret3(Mesh *pMesh)
{
    double StartTime, ExecTime, StartTime_;
    LARGE_INTEGER start, end, start_, end_;
    LARGE_INTEGER frequency;

    if (pTimer)
    {
        pTimer->Start();
        // StartTime = StartTime_ = pTimer->GetTime();

        // Get frequency
        QueryPerformanceFrequency((LARGE_INTEGER *)&frequency);

        QueryPerformanceCounter((LARGE_INTEGER *)&start);
        QueryPerformanceCounter((LARGE_INTEGER *)&start_);
    }

    //// Segment mesh to surfels.

    pSurfels->Init(pMesh);

    pSurfelDetector->Init(pMesh, pSurfels, pMem);

    printf("Segmentation to surfels... ");

    if (pSurfelDetector->pTimer)
        StartTime = pSurfelDetector->pTimer->GetTime();

    pSurfelDetector->Segment(pMesh, pSurfels);

    if (pSurfelDetector->pTimer)
        ExecTime = pSurfelDetector->pTimer->GetTime() - StartTime;

    printf("completed.\n");
    printf("No. of surfels = %d\n", pSurfels->NodeArray.n);

    if (pSurfelDetector->pTimer)
        printf("Total segmentation time = %lf s\n", ExecTime);

    pSurfels->DetectVertices(pMesh);

    // pSurfels->DetectOcclusionVertices(pMesh, camera); //commented for PR18 Experiment 5 - NEW BASELINE

    if (pTimer)
    {
        pTimer->Stop();

        surfelTime = pTimer->GetTime();

        pTimer->Start();

        // surfelTime = pTimer->GetTime() - StartTime_;

        // StartTime_ = pTimer->GetTime();

        // HPC timer
        QueryPerformanceCounter((LARGE_INTEGER *)&end_);

        surfelTime2 = (end_.QuadPart - start_.QuadPart) * 1000.0 / frequency.QuadPart;

        printf("surfelTime2=%lf\n", surfelTime2);

        QueryPerformanceCounter((LARGE_INTEGER *)&start_);
    }

    //// Detection of planar and convex surfaces

    // Detect planar surfaces.

    DetectSupportingSurfaces(pMesh);

    convexClustering.clusteringSurfelFlagMask = RVLSURFEL_FLAG_GND;
    convexClustering.clusteringSurfelFlags = 0;

    // Cluster surfels into convex and concave surfaces.

    bool bConcavity, bTorus;

    bConcavity = false;
    bTorus = false;

    RVL_DELETE_ARRAY(SClusters.Element);

    RVL_DELETE_ARRAY(clusterMap);

    clusterMap = new int[pSurfels->NodeArray.n];

    int nSCClusters, nSUClusters;

    // int *clusterMapAll = new int[pSurfels->NodeArray.n];

    PSGM_::ConvexAndConcaveClusters(pMesh, pSurfels, pSurfelDetector, &convexClustering, &concaveClustering,
                                    SClusters, nSCClusters, nSUClusters, convexClustering.minClusterSize, clusterMap, maxnSCClusters, maxnSUClusters, bConcavity, (bTorus ? maxnSTClusters : 0),
                                    visualizationData.bVisualizeConvexClusters, visualizationData.bVisualizeConcaveClusters, visualizationData.bVisualizeSurfels);

    RECOG::CreateObjectGraphFromSceneClusters(SClusters, pObjects, pSurfels, &unassignedSurfels, 50, RVLSURFEL_FLAG_GND, 0);

    int nSegments = pObjects->objectArray.n;

    pObjects->GetVertices();

    if (pTimer)
    {
        pTimer->Stop();

        planarAndConvexSurfacesTime = pTimer->GetTime();

        pTimer->Start();

        // planarAndConvexSurfacesTime = pTimer->GetTime() - StartTime_;

        // StartTime_ = pTimer->GetTime();

        // HPC timer
        QueryPerformanceCounter((LARGE_INTEGER *)&end_);

        planarAndConvexSurfacesTime2 = (end_.QuadPart - start_.QuadPart) * 1000.0 / frequency.QuadPart;

        printf("planarAndConvexSurfacesTime2=%lf\n", planarAndConvexSurfacesTime2);

        QueryPerformanceCounter((LARGE_INTEGER *)&start_);
    }

    //// Hypothesis generation and LEVEL 1 evaluation.

    // Initialize VNClassifier.

    Init(pMesh);

    // Initialize PSGM shape instance detection tool.

    pShapeInstanceDetection->pMesh = pMesh;

    if (bUseColor)
    {
        bool bUseChFilter = true;

        // Calculate surfel color descriptors
        int binsize[3] = {4, 4, 0};
        int chFilterThr[3] = {0, 4, 0};
        pSurfels->CalculateSurfelsColorHistograms(pShapeInstanceDetection->pMesh, RVLColorDescriptor::ColorSpaceList::HSV, false, binsize, true, chFilterThr, bUseChFilter);

        // ICL Dataset - test
        // int binsize[3] = { 2, 2, 2 };
        // int chFilterThr[3] = { 0, 0, 0 };
        // pSurfels->CalculateSurfelsColorHistograms(pShapeInstanceDetection->pMesh, RVLColorDescriptor::ColorSpaceList::RGB, false, binsize, true, chFilterThr, bUseChFilter);

        // One dimensional
        // pSurfels->CalculateSurfelsColorHistograms(pShapeInstanceDetection->pMesh, RVLColorDescriptor::ColorSpaceList::HSV, true, binsize, true, chFilterThr, true);
    }

    /// Create supersegments.

    CreateSuperSegments(pShapeInstanceDetection->minClusterSize);

    if (superSegments.n == 0)
    {
        printf("No valid supersegments!\n");

        // delete[] clusterMapAll;
        FreeTmpMem();

        return;
    }

    printf("No. of supersegments = %d\n", superSegments.n);

    int iSuperSegment;

    ///

    // Print super segment data on the screen.

    // for (iSuperSegment = 0; iSuperSegment < superSegments.n; iSuperSegment++)
    //{
    //	printf("H%d: ", iSuperSegment);

    //	pSuperSegment = superSegments.Element[iSuperSegment];

    //	for (i = 0; i < pSuperSegment->segments.n; i++)
    //		printf("%d ", pSuperSegment->segments.Element[i]);

    //	printf("\n");
    //}

    // Compute scene CTIs.

    float *DS = new float[superSegments.n * sampledUnitSphere.h];
    float *dS = DS;
    uchar *bDS = new uchar[superSegments.n * sampledUnitSphere.h];
    uchar *bdS = bDS;
    float *NS = new float[sampledUnitSphere.h * sampledUnitSphere.w];
    int *iV = new int[sampledUnitSphere.h];

    // NHull is not used
    Array<SURFEL::NormalHullElement> NHull; // not used
    NHull.Element = NULL;
    NHull.n = 0;

    VN_::SuperSegment *pSuperSegment;

    for (iSuperSegment = 0; iSuperSegment < superSegments.n; iSuperSegment++)
    {
        pSuperSegment = superSegments.Element[iSuperSegment];

        ComputeDescriptor(pSuperSegment->iVertexArray, NHull, pSuperSegment->pose.R, dS, bdS, NS, iV, pSuperSegment->type);

        dS += sampledUnitSphere.h;
        bdS += sampledUnitSphere.h;
    }

    // FILE *fpDS = fopen("C:\\RVL\\ExpRez\\DS.txt", "w");

    // PrintMatrix<float>(fpDS, DS, superSegments.n, sampledUnitSphere.h);

    // fclose(fpDS);

    // Compute scene bounding box.

    iObjectVertexArray.Element = new int[pSurfels->vertexArray.n];

    iObjectVertexArray.n = 0;

    Box<float> sceneBBox;

    SURFEL::Object *pSegment = pObjects->objectArray.Element;

    int iVertex = pSegment->iVertexArray.Element[0];

    // pSuperSegment = superSegments.Element[0];

    // int iVertex = pSuperSegment->iVertexArray.Element[0];

    InitBoundingBox<float>(&sceneBBox, pSurfels->vertexArray.Element[iVertex]->P);

    float *P;

    // for (iSuperSegment = 0; iSuperSegment < superSegments.n; iSuperSegment++)
    //{
    //	pSuperSegment = superSegments.Element[iSuperSegment];

    //	for (i = 0; i < pSuperSegment->iVertexArray.n; i++)
    //	{
    //		iVertex = pSuperSegment->iVertexArray.Element[i];

    //		if (!bVertexInArray[iVertex])
    //		{
    //			bVertexInArray[iVertex] = true;

    //			iObjectVertexArray.Element[iObjectVertexArray.n++] = iVertex;
    //		}

    //		P = pSurfels->vertexArray.Element[iVertex]->P;

    //		UpdateBoundingBox<float>(&sceneBBox, P);
    //	}
    //}

    int i;
    int iSegment;

    for (iSegment = 0; iSegment < nSegments; iSegment++)
    {
        pSegment = pObjects->objectArray.Element + iSegment;

        if (pSegment->flags & RVLPCSEGMENT_OBJECT_FLAG_GND)
            continue;

        for (i = 0; i < pSegment->iVertexArray.n; i++)
        {
            iVertex = pSegment->iVertexArray.Element[i];

            if (!bVertexInArray[iVertex])
            {
                bVertexInArray[iVertex] = true;

                iObjectVertexArray.Element[iObjectVertexArray.n++] = iVertex;
            }

            P = pSurfels->vertexArray.Element[iVertex]->P;

            UpdateBoundingBox<float>(&sceneBBox, P);
        }
    }

    for (i = 0; i < iObjectVertexArray.n; i++)
        bVertexInArray[iObjectVertexArray.Element[i]] = false;

    ExpandBox<float>(&sceneBBox, maxBoundingSphereRadius);

    // Compute scene centroid.

    // RVLNULL3VECTOR(Pc);
    //
    // iVertexArray.n = 0;

    // for (iSuperSegment = 0; iSuperSegment < superSegments.n; iSuperSegment++)
    //{
    //	pSuperSegment = superSegments.Element[iSuperSegment];

    //	for (i = 0; i < pSuperSegment->iVertexArray.n; i++)
    //	{
    //		iVertex = pSuperSegment->iVertexArray.Element[i];

    //		if (bVertexInArray[iVertex])
    //			continue;

    //		iVertexArray.Element[iVertexArray.n++] = iVertex;

    //		bVertexInArray[iVertex] = true;

    //		P = pSurfels->vertexArray.Element[iVertex]->P;

    //		RVLSUM3VECTORS(Pc, P, Pc);
    //	}
    //}

    // for (i = 0; i < iVertexArray.n; i++)
    //	bVertexInArray[iVertexArray.Element[i]] = false;

    // float fn = (float)(iVertexArray.n);

    // RVLSCALE3VECTOR2(Pc, fn, Pc);

    // Compute dilated depth image.

    pShapeInstanceDetection->CreateDilatedDepthImage();

    // Initialize hypothesis space.

    Space3DGrid<VN_::Hypothesis2, float> HSpace;

    float HSpaceCellSize = 0.02f;

    int maxnMatches = superSegments.n * maxnModelSegments;

    HSpace.Create((int)ceil((sceneBBox.maxx - sceneBBox.minx) / HSpaceCellSize),
                  (int)ceil((sceneBBox.maxy - sceneBBox.miny) / HSpaceCellSize),
                  (int)ceil((sceneBBox.maxz - sceneBBox.minz) / HSpaceCellSize),
                  HSpaceCellSize, maxnMatches);

    HSpace.SetVolume(sceneBBox.minx, sceneBBox.miny, sceneBBox.minz);

    // Allocate memory for sceneSegmentMatches.

    // RVL_DELETE_ARRAY(pShapeInstanceDetection->sceneSegmentMatches.Element);

    // pShapeInstanceDetection->sceneSegmentMatches.Element = new Array<SortIndex<float>>[superSegments.n];

    // pShapeInstanceDetection->sceneSegmentMatches.n = superSegments.n;

    // int nMatches = superSegments.n * pShapeInstanceDetection->MCTISet.pCTI.n;

    // if (nMatches > pShapeInstanceDetection->sceneSegmentMatchesArray.n)
    //{
    //	RVL_DELETE_ARRAY(pShapeInstanceDetection->sceneSegmentMatchesArray.Element);

    //	pShapeInstanceDetection->sceneSegmentMatchesArray.n = nMatches;

    //	pShapeInstanceDetection->sceneSegmentMatchesArray.Element = new SortIndex<float>[pShapeInstanceDetection->sceneSegmentMatchesArray.n];
    //}

    // RVL_DELETE_ARRAY(pShapeInstanceDetection->bestSceneSegmentMatches.Element);

    // pShapeInstanceDetection->bestSceneSegmentMatches.Element = new Array<SortIndex<float>>[superSegments.n];

    // pShapeInstanceDetection->bestSceneSegmentMatches.n = superSegments.n;

    // int nBestMatchesTotal = pShapeInstanceDetection->nBestMatchesPerCluster * superSegments.n;

    // if (nBestMatchesTotal > pShapeInstanceDetection->bestSceneSegmentMatchesArray.n)
    //{
    //	RVL_DELETE_ARRAY(pShapeInstanceDetection->bestSceneSegmentMatchesArray.Element);

    //	pShapeInstanceDetection->bestSceneSegmentMatchesArray.n = nBestMatchesTotal;

    //	pShapeInstanceDetection->bestSceneSegmentMatchesArray.Element = new SortIndex<float>[pShapeInstanceDetection->bestSceneSegmentMatchesArray.n];
    //}

    printf("Hypothesis generation...");

    // Parameters.

    // Moved to cfg file!
    // float edThr = 0.6f;

    //

    /*float *A = new float[9 * superSegments.n];
    int *nV_ = new int[superSegments.n];

    float *A_;

    for (iSuperSegment = 0; iSuperSegment < superSegments.n; iSuperSegment++)
    {
        pSuperSegment = superSegments.Element[iSuperSegment];

        dS = DS + sampledUnitSphere.h * iSuperSegment;
        bdS = bDS + sampledUnitSphere.h * iSuperSegment;

        A_ = A + 9 * iSuperSegment;

        nV_[iSuperSegment] = InitFitLS(dS, bdS, A_);
    }*/

    QList<VN_::Hypothesis2> *pHypothesisList = &hypothesisList;

    RVLQLIST_INIT(pHypothesisList);

    float sigd12 = CTIMatchSigmad1 * CTIMatchSigmad1;
    // float sigd22 = CTIMatchSigmad2 * CTIMatchSigmad2;

    float *e = new float[sampledUnitSphere.h];

    int nHypotheses = 0;

    int nModels = pShapeInstanceDetection->MCTISet.nModels;

    // float fnCTIElements = (float)(sampledUnitSphere.h);

    int maxnModelPts = 0;

    int nMatches = 0;
    int nQualifiedMatches = 0;
    int nLevel1Hypotheses = 0;

    VN_::Hypothesis2 hypothesis;
    VN_::Hypothesis2 *pHypothesis;
    int iModel, modelID, iMCTI, nVisible;
    float *dM, *RLMM, *RLSS;
    uchar *bdM;
    float tLMLS[3], RMS[9], tMS[3];
    float E, e2, e2nrm;
    PSGM_::ModelInstance *pSCTI, *pMCTI;
    float t_[3];

    for (iModel = 0; iModel < pShapeInstanceDetection->activeModels.n; iModel++)
    {
        modelID = pShapeInstanceDetection->activeModels.Element[iModel];

        if (pShapeInstanceDetection->sampledModels.Element[iModel].n > maxnModelPts)
            maxnModelPts = pShapeInstanceDetection->sampledModels.Element[iModel].n;

        // Match scene CTIs with model CTIs.

        for (iSuperSegment = 0; iSuperSegment < superSegments.n; iSuperSegment++)
        {
            //////if (nV_[iSuperSegment] < 3)
            //////	continue;

            pSuperSegment = superSegments.Element[iSuperSegment];

            dS = DS + sampledUnitSphere.h * iSuperSegment;
            bdS = bDS + sampledUnitSphere.h * iSuperSegment;

            RLSS = pSuperSegment->pose.R;

            //////A_ = A + 9 * iSuperSegment;

            for (iMCTI = modelSegmentInterval[modelID].a; iMCTI < modelSegmentInterval[modelID].b; iMCTI++)
            {
                nMatches++;

                // if (iMCTI == 24)
                //	int debug = 0;

                pMCTI = pShapeInstanceDetection->MCTISet.pCTI.Element[iMCTI];

                dM = DM + sampledUnitSphere.h * iMCTI;
                bdM = bDM + sampledUnitSphere.h * iMCTI;

                int nV;

                if (FitLS(dM, bdM, dS, bdS, tLMLS, nV, e) > 0.0f)
                {
                    // FitLS(A_, dM, dS, bdS, tLMLS, e);

                    E = 0.0f;

                    nVisible = 0;

                    for (i = 0; i < sampledUnitSphere.h; i++)
                    {
                        if (dS[i] < 0.0f)
                        {
                            nVisible++;

                            e2 = e[i] * e[i];

                            e2nrm = e2 / sigd12;

                            if (!bdS[i])
                            {
                                if (e[i] < 0.0f)
                                {
                                    if (e2nrm > CTIMatchSigmad2)
                                        e2nrm = CTIMatchSigmad2;
                                }
                            }

                            E += (e2nrm <= 1.0f ? e2nrm : 1.0f);
                        }
                    }

                    E /= (float)nVisible;

                    if (E < edThr)
                    {
                        nQualifiedMatches++;

                        RLMM = pMCTI->R;

                        RVLMXMUL3X3T2(RLSS, RLMM, RMS);
                        RVLMULMX3X3VECT(RLSS, tLMLS, tMS);

                        // 190729

                        hypothesis.iMatch = nHypotheses;
                        RVLCOPY3VECTOR(tMS, hypothesis.P);
                        RVLCOPYMX3X3(RMS, hypothesis.R);
                        hypothesis.score = E;

                        pHypothesis = HSpace.AddData(hypothesis);

                        //

                        // if (pHypothesis = HSpace.Add3DPose(nHypotheses, RMS, tMS, E, CTIMatchRedundantPoseOrientThr))	// 190729
                        {
                            pHypothesis->idx = nHypotheses;
                            pHypothesis->flags = 0x00;
                            pHypothesis->pSuperSegment = pSuperSegment;
                            pHypothesis->iModel = modelID;
                            pHypothesis->iMCTI = iMCTI;
                            pHypothesis->CTIcost = E;
                            // RVLCOPY3VECTOR(tLMLS, pHypothesis->tLMLS);
                            pHypothesis->CHMatchingMetric = 10.0f;

                            nHypotheses++;

                            // if (pHypothesis->iCell == 8717)
                            //	int debug = 0;
                        }
                    }
                }
            }

            // HSpace.CopyData(pHypothesisList, pMem);	// 190729

            // HSpace.Clear();	// 190729
        } // for every supersegment

        // 	190729

        nLevel1Hypotheses += HSpace.Prune3DPoses(CTIMatchRedundantPoseOrientThr);

        HSpace.CopyData(pHypothesisList, sizeof(VN_::Hypothesis2), pMem);

        HSpace.Clear();

        //
    } // for every model

    delete[] NS;
    delete[] iV;
    delete[] e;
    // delete[] A;
    // delete[] nV_;

    printf("completed.\n");

    printf("Total number of matches = %d\n", nMatches);
    printf("Number of matches with CTI error below %f = %d\n", edThr, nQualifiedMatches);
    printf("Number of LEVEL1 hypotheses = %d\n", nLevel1Hypotheses);

    if (pTimer)
    {
        pTimer->Stop();

        hypGenAndLEVEL1Time = pTimer->GetTime();

        pTimer->Start();

        // hypGenAndLEVEL1Time = pTimer->GetTime() - StartTime_;

        // StartTime_ = pTimer->GetTime();

        // HPC timer
        QueryPerformanceCounter((LARGE_INTEGER *)&end_);

        hypGenAndLEVEL1Time2 = (end_.QuadPart - start_.QuadPart) * 1000.0 / frequency.QuadPart;

        QueryPerformanceCounter((LARGE_INTEGER *)&start_);
    }

    //// LEVEL2 Evaluation:

    printf("LEVEL2 hypothesis evaluation...");

    // Initialize Z-buffer.

    pShapeInstanceDetection->sceneSamplingResolution = 2;

    // pShapeInstanceDetection->SampleScene();

    pShapeInstanceDetection->InitZBuffer(pMesh);

    CreateImage3x3NeighborhoodLT(pShapeInstanceDetection->ZBuffer.w, pShapeInstanceDetection->image3x3Neighborhood);

    // Detect ground plane.

    // DetectGroundPlane(pMesh);

    // Create surfel mask with groundplane masked.

    VN_::SurfelMask sceneSurfelMask;

    CreateSurfelMask(pMesh, sceneSurfelMask);

    CreateForegroundObjectSurfelMask(sceneSurfelMask);

    // DisplaySurfelMask(sceneSurfelMask);

    pShapeInstanceDetection->surfelMask = sceneSurfelMask.mask;

    // Assign hypotheses to every supersegment.

    int nSuperSegmentHypothesisSets = superSegments.n * nModels;

    QList<QLIST::Ptr<VN_::Hypothesis2>> *superSegmentHypothesisList = new QList<QLIST::Ptr<VN_::Hypothesis2>>[nSuperSegmentHypothesisSets];

    QList<QLIST::Ptr<VN_::Hypothesis2>> *pSuperSegmentHypothesisList = superSegmentHypothesisList;

    Array<VN_::HypothesisSortPtr> *superSegmentHypotheses = new Array<VN_::HypothesisSortPtr>[nSuperSegmentHypothesisSets];

    RVL_DELETE_ARRAY(sortedSuperSegmentHypotheses);

    sortedSuperSegmentHypotheses = new Array<VN_::HypothesisSortPtr>[superSegments.n];

    Array<VN_::HypothesisSortPtr> *superSegmentHypotheses_ = superSegmentHypotheses;

    int iSuperSegmentHypothesisSet;

    for (iSuperSegmentHypothesisSet = 0; iSuperSegmentHypothesisSet < nSuperSegmentHypothesisSets; iSuperSegmentHypothesisSet++, pSuperSegmentHypothesisList++, superSegmentHypotheses_++)
    {
        RVLQLIST_INIT(pSuperSegmentHypothesisList);

        superSegmentHypotheses_->n = 0;
    }

    QLIST::Ptr<VN_::Hypothesis2> *superSegmentHypothesesMem = new QLIST::Ptr<VN_::Hypothesis2>[nHypotheses];

    QLIST::Ptr<VN_::Hypothesis2> *pHypothesisPtr = superSegmentHypothesesMem;

    pHypothesis = hypothesisList.pFirst;

    while (pHypothesis)
    {
        iSuperSegmentHypothesisSet = pHypothesis->pSuperSegment->ID * nModels + pHypothesis->iModel;

        pSuperSegmentHypothesisList = superSegmentHypothesisList + iSuperSegmentHypothesisSet;

        RVLQLIST_ADD_ENTRY(pSuperSegmentHypothesisList, pHypothesisPtr);

        pHypothesisPtr->ptr = pHypothesis;

        pHypothesisPtr++;

        superSegmentHypotheses[iSuperSegmentHypothesisSet].n++;

        pHypothesis = pHypothesis->pNext;
    }

    // Identify best nHypothesesLevel2 hypotheses for every supersegment.

    D = new float[3 * maxnModelCHFaces];
    NBuff = new float[3 * maxnModelCHFaces];

    int nHypothesesLevel2 = pShapeInstanceDetection->nBestMatchesPerCluster;

    int maxnSuperSegmentHypothesisSetSize = 0;

    for (iSuperSegmentHypothesisSet = 0; iSuperSegmentHypothesisSet < nSuperSegmentHypothesisSets; iSuperSegmentHypothesisSet++)
        if (superSegmentHypotheses[iSuperSegmentHypothesisSet].n > maxnSuperSegmentHypothesisSetSize)
            maxnSuperSegmentHypothesisSetSize = superSegmentHypotheses[iSuperSegmentHypothesisSet].n;

    VN_::HypothesisSortPtr *superSegmentHypothesesMem2 = new VN_::HypothesisSortPtr[nHypothesesLevel2 * nSuperSegmentHypothesisSets];
    VN_::HypothesisSortPtr *superSegmentHypothesesMem3 = new VN_::HypothesisSortPtr[nHypothesesLevel2 * nSuperSegmentHypothesisSets];

    Array<VN_::HypothesisSortPtr> superSegmentHypothesesBuff;

    superSegmentHypothesesBuff.Element = new VN_::HypothesisSortPtr[maxnSuperSegmentHypothesisSetSize];

    // new
    VN_::HypothesisSortPtr *pHypothesisSupesegmentPtr = superSegmentHypothesesMem2;
    VN_::HypothesisSortPtr *pHypothesisSupesegmentSortPtr = superSegmentHypothesesMem3;

    Array<int> TP;
    TP.Element = new int[maxnModelPts];
    Array<int> FP;
    FP.Element = new int[maxnModelPts];
    Array<int> FN;
    FN.Element = new int[maxnModelPts];

    int nFilteredHypotheses = 0;
    int nHypothesesEvaluatedByIP = 0;

    VN_::HypothesisSortPtr *pHypothesisSortPtr;
    float score, eCH2avg, eCH2max;
    int nVisiblePts, nTransparentPts, nLevel2Hypotheses;
    float gndPlaneDist;
    float wGndDistance12 = pShapeInstanceDetection->wGndDistance1 * pShapeInstanceDetection->wGndDistance1;

    for (iSuperSegment = 0; iSuperSegment < superSegments.n; iSuperSegment++)
    {
        sortedSuperSegmentHypotheses[iSuperSegment].Element = pHypothesisSupesegmentSortPtr;
        sortedSuperSegmentHypotheses[iSuperSegment].n = 0;

        for (iModel = 0; iModel < nModels; iModel++)
        {
            iSuperSegmentHypothesisSet = iSuperSegment * nModels + iModel;

            pSuperSegmentHypothesisList = superSegmentHypothesisList + iSuperSegmentHypothesisSet;

            pHypothesisSortPtr = superSegmentHypothesesBuff.Element;

            pHypothesisPtr = pSuperSegmentHypothesisList->pFirst;

            while (pHypothesisPtr)
            {
                pHypothesis = pHypothesisPtr->ptr;

                gndPlaneDist = PointSetToPlaneDistance(modelCH[pHypothesis->iModel]->NodeArray, pHypothesis->R, pHypothesis->P, NGnd, dGnd);

                pHypothesis->gndPlaneDistance = gndPlaneDist;

                // if (pHypothesis->iModel == 16)
                //	int debug = 0;

                // filter hypotheses using gndPlaneDistance
                // if (RVLABS(gndPlaneDist) < pShapeInstanceDetection->gndDistanceThresh) //BASELINE
                if (gndPlaneDist > -pShapeInstanceDetection->gndDistanceThresh)
                {
                    pHypothesisSortPtr->ptr = pHypothesis;
                    pHypothesisSortPtr->cost = pHypothesis->score;

                    // HypothesisEvaluationCH(pHypothesis, eCH2avg, eCH2max);

                    // pHypothesisSortPtr->cost = eCH2avg;

                    score = pShapeInstanceDetection->HypothesisEvaluationIP(pShapeInstanceDetection->sampledModels.Element[pHypothesis->iModel], pHypothesis->R, pHypothesis->P, 0.03f,
                                                                            nVisiblePts, nTransparentPts, &TP, &FP, &FN);

                    // Experiment 9
                    // if (pHypothesis->idx == 956)
                    //{
                    //	int *SMCorrespondence = new int[pShapeInstanceDetection->ZBuffer.w * pShapeInstanceDetection->ZBuffer.h];

                    //	score = pShapeInstanceDetection->HypothesisEvaluationIP2(pShapeInstanceDetection->modelPCs[pHypothesis->iModel], pHypothesis->R, pHypothesis->P, 1.0f, 0.01f,
                    //		nTransparentPts, SMCorrespondence);

                    //	pShapeInstanceDetection->DisplayHypothesisEvaluationIP2(visualizationData.pVisualizer, SMCorrespondence, nTransparentPts, visualizationData.selectedHypothesisActor);

                    //	delete[] SMCorrespondence;
                    //}

                    // pHypothesisSortPtr->cost = (nVisiblePts > 0 ? 1.0f + (-score + (float)(nTransparentPts)) / (float)nVisiblePts : 2.0f);
                    // pHypothesisSortPtr->cost = (nVisiblePts > 0 ? -score + (float)(nTransparentPts) : nVisiblePts); //NEW
                    if (gndPlaneDist > pShapeInstanceDetection->gndDistanceThresh)
                        gndPlaneDist = pShapeInstanceDetection->gndDistanceThresh;

                    pHypothesisSortPtr->cost = (nVisiblePts > 0 ? -score * (1.0f - pShapeInstanceDetection->wGndDistance1 * RVLABS(gndPlaneDist)) + (float)(nTransparentPts) : nVisiblePts); // NEW

                    pHypothesis->matchedPtsPercentage = (float)score / nVisiblePts;

                    // if (pHypothesisSortPtr->cost > 3.0f)
                    //	int debug = 0;

                    // if (iSuperSegment == 8 && pHypothesisSortPtr - superSegmentHypothesesBuff.Element == 216)
                    //	int debug = 0;

                    pHypothesis->score = pHypothesisSortPtr->cost;

                    pHypothesisSortPtr++;

                    nHypothesesEvaluatedByIP++;
                }
                else
                    nFilteredHypotheses++;

                pHypothesisPtr = pHypothesisPtr->pNext;
            }

            superSegmentHypothesesBuff.n = pHypothesisSortPtr - superSegmentHypothesesBuff.Element;

            // superSegmentHypotheses[iSuperSegmentHypothesisSet].Element = superSegmentHypothesesMem2 + iSuperSegmentHypothesisSet * nHypothesesLevel2;
            superSegmentHypotheses[iSuperSegmentHypothesisSet].Element = pHypothesisSupesegmentPtr; // NEW

            nLevel2Hypotheses = RVLMIN(nHypothesesLevel2, superSegmentHypothesesBuff.n);

            pHypothesisSupesegmentPtr += nLevel2Hypotheses;     // NEW
            pHypothesisSupesegmentSortPtr += nLevel2Hypotheses; // NEW

            sortedSuperSegmentHypotheses[iSuperSegment].n += nLevel2Hypotheses; // NEW

            // Min<VN_::HypothesisSortPtr, float>(superSegmentHypothesesBuff, nLevel2Hypotheses, superSegmentHypotheses[iSuperSegment]);

            // for (i = 0; i < superSegmentHypotheses[iSuperSegmentHypothesisSet].n; i++)
            //	superSegmentHypotheses[iSuperSegmentHypothesisSet].Element[i].ptr->flags |= RVLVN_HYPOTHESIS2_FLAG_LEVEL1;

            BubbleSort<VN_::HypothesisSortPtr>(superSegmentHypothesesBuff);

            superSegmentHypotheses_ = superSegmentHypotheses + iSuperSegmentHypothesisSet;

            superSegmentHypotheses_->n = 0;

            for (i = 0; i < superSegmentHypothesesBuff.n; i++)
            {
                superSegmentHypothesesBuff.Element[i].ptr->rank = i;

                if (i < nLevel2Hypotheses)
                    superSegmentHypotheses_->Element[superSegmentHypotheses_->n++] = superSegmentHypothesesBuff.Element[i];
            }
        }
    }

    delete[] superSegmentHypothesesBuff.Element;

    printf("completed.\n");

    printf("Number of hypotheses filtered by ground distance: %d (filtered: %.2f)\n", nFilteredHypotheses, (float)nFilteredHypotheses / (float)nLevel1Hypotheses);
    printf("Number of hypotheses evaluated by image projection: %d\n", nHypothesesEvaluatedByIP);

    // Sort hypotheses for every super segment - Vidovic
    printf("Sort hypotheses for every supersegment...");

    // copy superSegmentHypothesesMem2 to superSegmentHypothesesMem3
    memcpy(superSegmentHypothesesMem3, superSegmentHypothesesMem2, sizeof(VN_::HypothesisSortPtr) * nHypothesesLevel2 * nSuperSegmentHypothesisSets);

    // sort hypotheses for every supersegment
    for (iSuperSegment = 0; iSuperSegment < superSegments.n; iSuperSegment++)
        BubbleSort<VN_::HypothesisSortPtr>(sortedSuperSegmentHypotheses[iSuperSegment]);

    printf("completed.\n");

// sort hypotheses for every model
#ifdef NEVER
    printf("Sort hypotheses for every model...");

    int nHypothesesPerModel = 5;

    int nModelHypotheses;

    RVL_DELETE_ARRAY(sortedModelHypotheses);

    sortedModelHypotheses = new Array<VN_::HypothesisSortPtr>[nModels];

    sortedModelHypotheses->n = nModels;

    Array<VN_::HypothesisSortPtr> *sortedModelHypotheses_ = sortedModelHypotheses;

    VN_::HypothesisSortPtr *superSegmentHypothesesMem4 = new VN_::HypothesisSortPtr[nHypothesesPerModel * nModels];

    VN_::HypothesisSortPtr *pHypothesisModelPtr = superSegmentHypothesesMem4;

    int maxnModelHypotheses = nHypothesesLevel2 * superSegments.n;

    Array<VN_::HypothesisSortPtr> modelHypothesesBuff;

    VN_::HypothesisSortPtr *pHypothesisModelSortPtr;

    modelHypothesesBuff.Element = new VN_::HypothesisSortPtr[maxnModelHypotheses];

    for (iModel = 0; iModel < nModels; iModel++)
    {
        sortedModelHypotheses[iModel].Element = pHypothesisModelPtr;

        pHypothesisModelSortPtr = modelHypothesesBuff.Element;

        for (iSuperSegment = 0; iSuperSegment < superSegments.n; iSuperSegment++)
        {
            iSuperSegmentHypothesisSet = iSuperSegment * nModels + iModel;

            for (int iHypothesis = 0; iHypothesis < superSegmentHypotheses[iSuperSegmentHypothesisSet].n; iHypothesis++)
            {
                pHypothesis = superSegmentHypotheses[iSuperSegmentHypothesisSet].Element[iHypothesis].ptr;

                pHypothesisModelSortPtr->ptr = pHypothesis;
                pHypothesisModelSortPtr->cost = pHypothesis->score;

                pHypothesisModelSortPtr++;
            }
        }

        nModelHypotheses = pHypothesisModelSortPtr - modelHypothesesBuff.Element;

        modelHypothesesBuff.n = nModelHypotheses;

        BubbleSort<VN_::HypothesisSortPtr>(modelHypothesesBuff);

        nModelHypotheses = RVLMIN(nHypothesesPerModel, modelHypothesesBuff.n);

        pHypothesisModelPtr += nModelHypotheses;

        sortedModelHypotheses_ = sortedModelHypotheses + iModel;

        sortedModelHypotheses_->n = 0;

        for (i = 0; i < modelHypothesesBuff.n; i++)
        {
            modelHypothesesBuff.Element[i].ptr->rank = i;

            if (i < nModelHypotheses)
                sortedModelHypotheses_->Element[sortedModelHypotheses_->n++] = modelHypothesesBuff.Element[i];
        }
    }

    RVL_DELETE_ARRAY(modelHypothesesBuff.Element);

    printf("completed.\n");
#endif

    if (pTimer)
    {
        pTimer->Stop();

        LEVEL2Time = pTimer->GetTime();

        pTimer->Start();

        // LEVEL2Time = pTimer->GetTime() - StartTime_;

        // StartTime_ = pTimer->GetTime();

        // HPC timer
        QueryPerformanceCounter((LARGE_INTEGER *)&end_);

        LEVEL2Time2 = (end_.QuadPart - start_.QuadPart) * 1000.0 / frequency.QuadPart;

        QueryPerformanceCounter((LARGE_INTEGER *)&start_);
    }

    //// LEVEL3 Evaluation:
    // ===========================================================================================
    // Apply fast ICP to the first nHypothesesLevel3 hypotheses of every supersegment.
    // Assign segments to hypotheses and create a surfel mask for each hypothesis.
    // Compute hypothesis scores.
    // Re-sort the hypotheses.

    printf("ICP...");

    int avgNoSegmentsPerHypothesis = 3;

    VN_::SurfelMask objectSurfelMask;

    CreateSurfelMask(pMesh, objectSurfelMask);

    CRVLMem segmentMem;
    CRVLMem *pSegmentMem = &segmentMem;

    segmentMem.Create(superSegments.n * nModels * nHypothesesLevel3 * avgNoSegmentsPerHypothesis * sizeof(int));

    int nICPFittings = 0;

    int j, nLevel3Hypotheses, iTmp;
    Array<OrientedPoint> modelPoints;

    float CHDist;

    VN_::Hypothesis2 *pHypothesis_;

    for (iSuperSegment = 0; iSuperSegment < superSegments.n; iSuperSegment++)
    {
        // if (iSuperSegment == 138)
        //	int debug = 0;

        if (!bUseLevel3MatchedPercentageThr)
            nLevel3Hypotheses = RVLMIN(nHypothesesLevel3, sortedSuperSegmentHypotheses[iSuperSegment].n);
        else
        {
            if (sortedSuperSegmentHypotheses[iSuperSegment].n <= nHypothesesLevel3)
                nLevel3Hypotheses = sortedSuperSegmentHypotheses[iSuperSegment].n;
            else
            {
                nLevel3Hypotheses = nHypothesesLevel3;

                // start from nHypothesesLevel3 hypothesis and check all hypotheses
                // find all hypotheses with matchedPtsPercentage greater than level3MatchedPercentageThr which are ranked lower than nHypothesesLevel3
                for (i = nHypothesesLevel3; i < sortedSuperSegmentHypotheses[iSuperSegment].n; i++)
                {
                    pHypothesis = sortedSuperSegmentHypotheses[iSuperSegment].Element[i].ptr;

                    if (pHypothesis->matchedPtsPercentage >= level3MatchedPercentageThr)
                    {
                        // move selected hypothesis to the end of level3Hypotheses list
                        pHypothesis_ = sortedSuperSegmentHypotheses[iSuperSegment].Element[nLevel3Hypotheses].ptr;

                        sortedSuperSegmentHypotheses[iSuperSegment].Element[nLevel3Hypotheses].ptr = sortedSuperSegmentHypotheses[iSuperSegment].Element[i].ptr;

                        sortedSuperSegmentHypotheses[iSuperSegment].Element[i].ptr = pHypothesis;

                        nLevel3Hypotheses++;
                    }
                }
            }
        }

        for (i = 0; i < nLevel3Hypotheses; i++)
        {
            pHypothesis = sortedSuperSegmentHypotheses[iSuperSegment].Element[i].ptr;

            pShapeInstanceDetection->surfelMask = sceneSurfelMask.mask;

            modelPoints = pShapeInstanceDetection->sampledModels.Element[pHypothesis->iModel];

            if (pShapeInstanceDetection->bHollow)
                if (pShapeInstanceDetection->bHollow[pHypothesis->iModel])
                    modelPoints.n /= 2;

            // PR18 for Experiment 6 - CHAL algorithm evaluation
            memcpy(pHypothesis->R_CHAL, pHypothesis->R, 9 * sizeof(float));
            memcpy(pHypothesis->P_CHAL, pHypothesis->P, 3 * sizeof(float));

            pShapeInstanceDetection->ICP(modelPoints, pHypothesis->R, pHypothesis->P, 0.04f, 5, pHypothesis->R, pHypothesis->P);

            RVLMEM_ALLOC_STRUCT_ARRAY(pSegmentMem, int, nSegments, pHypothesis->segments.Element);

            pHypothesis->segments.n = 0;

            AssignSegmentsToHypothesis(pHypothesis, hypothesisEvaluationLevel3CHTolerance);

            segmentMem.m_pFreeMem = (BYTE *)(pHypothesis->segments.Element + pHypothesis->segments.n);

            CreateSurfelMask(pHypothesis->segments, objectSurfelMask);

            pShapeInstanceDetection->surfelMask = objectSurfelMask.mask;

            // if (i == 0)
            //	DisplaySurfelMask(objectSurfelMask);

            if (hypothesisEvaluationLevel3Method == 1)
                score = pShapeInstanceDetection->HypothesisEvaluationIP(pShapeInstanceDetection->sampledModels.Element[pHypothesis->iModel], pHypothesis->R, pHypothesis->P, 0.03f,
                                                                        nVisiblePts, nTransparentPts, &TP, &FP, &FN);
            else if (hypothesisEvaluationLevel3Method == 2)
            {
                score = pShapeInstanceDetection->HypothesisEvaluationIP2(pShapeInstanceDetection->modelPCs[pHypothesis->iModel], pHypothesis->R, pHypothesis->P, 1.0f, 0.01f, nTransparentPts);

                nVisiblePts = pShapeInstanceDetection->ZBufferActivePtArray.n;
            }

            CreateSurfelMask(pHypothesis->segments, objectSurfelMask, false);

            gndPlaneDist = pHypothesis->gndPlaneDistance = PointSetToPlaneDistance(modelCH[pHypothesis->iModel]->NodeArray, pHypothesis->R, pHypothesis->P, NGnd, dGnd);

            if (gndPlaneDist > pShapeInstanceDetection->gndDistanceThresh)
                gndPlaneDist = pShapeInstanceDetection->gndDistanceThresh;

            if (bUseColor)
            {
                // determine surfels contained in pHypothesis pose
                pObjects->UnionOfSurfels(pHypothesis->segments, iSurfelArray, bSurfelInArray);

                // calculate CH matching score
                CHDist = pHypothesis->CHMatchingMetric = CHMatching(iSurfelArray, pHypothesis->iModel, RVLColorDescriptor::MetricsList::Bhattacharyya);

                pHypothesis->score = sortedSuperSegmentHypotheses[iSuperSegment].Element[i].cost =
                    (nVisiblePts > 0 ? -score * (1.0f - pShapeInstanceDetection->wGndDistance1 * RVLABS(gndPlaneDist) - pShapeInstanceDetection->wColor * CHDist) + (float)(nTransparentPts) : nVisiblePts);
            }
            else
                pHypothesis->score = sortedSuperSegmentHypotheses[iSuperSegment].Element[i].cost =
                    (nVisiblePts > 0 ? -score * (1.0f - pShapeInstanceDetection->wGndDistance1 * RVLABS(gndPlaneDist)) + (float)(nTransparentPts) : nVisiblePts);

            pHypothesis->matchedPtsPercentage = (float)score / nVisiblePts;

            nICPFittings++;
        }

        iTmp = sortedSuperSegmentHypotheses[iSuperSegment].n;

        sortedSuperSegmentHypotheses[iSuperSegment].n = nLevel3Hypotheses;

        BubbleSort<VN_::HypothesisSortPtr>(sortedSuperSegmentHypotheses[iSuperSegment]);

        sortedSuperSegmentHypotheses[iSuperSegment].n = iTmp;
    }

    printf("completed.\n");

    delete[] TP.Element;
    delete[] FP.Element;
    delete[] FN.Element;

    if (pTimer)
    {
        pTimer->Stop();

        LEVEL3Time = pTimer->GetTime();

        pTimer->Start();

        // LEVEL3Time = pTimer->GetTime() - StartTime_;

        // StartTime_ = pTimer->GetTime();

        // HPC timer
        QueryPerformanceCounter((LARGE_INTEGER *)&end_);

        LEVEL3Time2 = (end_.QuadPart - start_.QuadPart) * 1000.0 / frequency.QuadPart;

        QueryPerformanceCounter((LARGE_INTEGER *)&start_);
    }

    //// Create interpretation by greedy search.

    GreedyInterpretation2();

    delete[] sceneSurfelMask.mem;
    delete[] objectSurfelMask.mem;

    ////

    if (pTimer)
    {
        pTimer->Stop();

        greedyInterpretationTime = pTimer->GetTime();

        // greedyInterpretationTime = pTimer->GetTime() - StartTime_;

        // ExecTime = pTimer->GetTime();

        // HPC timer
        QueryPerformanceCounter((LARGE_INTEGER *)&end_);

        // for measuring total time
        QueryPerformanceCounter((LARGE_INTEGER *)&end);

        greedyInterpretationTime2 = (end_.QuadPart - start_.QuadPart) * 1000.0 / frequency.QuadPart;

        // calculate total time
        totalTime = surfelTime + planarAndConvexSurfacesTime + hypGenAndLEVEL1Time + LEVEL2Time + LEVEL3Time + greedyInterpretationTime;

        // totalTime_ = ExecTime - StartTime;

        totalTime2 = surfelTime2 + planarAndConvexSurfacesTime2 + hypGenAndLEVEL1Time2 + LEVEL2Time2 + LEVEL3Time2 + greedyInterpretationTime2;

        totalTime2_ = (end.QuadPart - start.QuadPart) * 1000.0 / frequency.QuadPart;

        printf("Total execution time = %lf ms\n", totalTime);

        FILE *fpTime = fopen((std::string(resultsFolder) + "\\time.txt").data(), "a");

        fprintf(fpTime, "%02d\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", sceneID, surfelTime, planarAndConvexSurfacesTime, hypGenAndLEVEL1Time, LEVEL2Time, LEVEL3Time, greedyInterpretationTime, totalTime, 0.0f);

        fprintf(fpTime, "%02d\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", sceneID, surfelTime2, planarAndConvexSurfacesTime2, hypGenAndLEVEL1Time2, LEVEL2Time2, LEVEL3Time2, greedyInterpretationTime2, totalTime2, totalTime2_);

        fclose(fpTime);
    }

    /*if (pTimer)
    {
        pTimer->Stop();

        ExecTime = pTimer->GetTime();

        printf("Total execution time = %lf ms\n", ExecTime);

        FILE *fpTime = fopen((std::string(resultsFolder) + "\\time.txt").data(), "a");

        //fprintf(fpTime, "%02d\t%lf\n", sceneID, ExecTime);

        fprintf(fpTime, "%lf\n", ExecTime); //for Experiment 8

        fclose(fpTime);
    }*/

    // Evaluate interpretation.

    int *iGTModel = NULL;
    VN_::GTHypothesis *GTHypothesis = NULL;

    Array<GTInstance> *pGTArray;
    GTInstance *pGT;
    int iGT;

    if (pShapeInstanceDetection->pECCVGT->GT.Element)
    {
        pGTArray = pShapeInstanceDetection->pECCVGT->GT.Element + sceneID;

        iGTModel = new int[nModels];

        memset(iGTModel, 0xff, nModels * sizeof(int));

        GTHypothesis = new VN_::GTHypothesis[pGTArray->n];

        for (iGT = 0; iGT < pGTArray->n; iGT++)
        {
            pGT = pGTArray->Element + iGT;

            iGTModel[pGT->iModel] = iGT;

            GTHypothesis[iGT].pHypothesis = NULL;
        }

        FILE *fpInterpretation = fopen((std::string(resultsFolder) + "\\interpretation.txt").data(), "a");

        fprintf(fpInterpretation, "scene %d\n\n", sceneID);

        // PR18 for Experiment 6 - CHAL algorithm evaluation
        FILE *fpCHAL = fopen((std::string(resultsFolder) + "\\CHALEvaluation.txt").data(), "a");

        // Save RMSE for CHAL pose and ICP pose
        FILE *fpRMSE = fopen((std::string(resultsFolder) + "\\RMSE.dat").data(), "a");

        // Save RMSE for CHAL pose and ICP pose
        FILE *fpTP = fopen((std::string(resultsFolder) + "\\TP.dat").data(), "a");
        FILE *fpFP = fopen((std::string(resultsFolder) + "\\FP.dat").data(), "a");

        char strTP[] = "TP";
        char strFP[] = "FP";

        char *strHypothesisEval;
        double TMS[16];
        double TMS_CHAL[16];
        float RGT[9];
        float tGT[3];
        double TGT[16];
        float RMSE_;
        float RMSE_CHAL;

        int nTP_ = 0, nFP_ = 0, nFN_ = 0;
        float precision_, recall_;

        // for (i = 0; i < interpretation2.size(); i++)
        //{
        //	pHypothesis = interpretation2[i];

        //	iGT = iGTModel[pHypothesis->iModel];

        //	if (iGT >= 0)
        //	{
        //		pGT = pGTArray->Element + iGT;

        //		RVLHTRANSFMX(pHypothesis->R, pHypothesis->P, TMS);
        //		RVLCOPYMX3X3(pGT->R, RGT);
        //		RVLCOPY3VECTOR(pGT->t, tGT);
        //		RVLHTRANSFMX(RGT, tGT, TGT);

        //		RMSE_ = pShapeInstanceDetection->RMSE(pHypothesis->iModel, TGT, TMS);

        //		if (RMSE_ <= 0.03f)
        //		{
        //			strHypothesisEval = strTP;

        //			GTHypothesis[iGT].pHypothesis = pHypothesis;

        //			nTP_++;

        //			//PR18 for Experiment 6 - CHAL algorithm evaluation
        //			fprintf(fpCHAL, "%d\t%07d\t%02d\t%02d\t",
        //				sceneID, pHypothesis->idx, pHypothesis->iModel, pHypothesis->pSuperSegment->ID);

        //			for (j = 0; j < 9; j++)
        //				fprintf(fpCHAL, "%f\t", pHypothesis->R_CHAL[j]);

        //			for (j = 0; j < 3; j++)
        //				fprintf(fpCHAL, "%f\t", pHypothesis->P_CHAL[j]);

        //			for (j = 0; j < 9; j++)
        //				fprintf(fpCHAL, "%f\t", pHypothesis->R[j]);

        //			for (j = 0; j < 3; j++)
        //				fprintf(fpCHAL, "%f\t", pHypothesis->P[j]);

        //			fprintf(fpCHAL, "\n");

        //			//For RMSE NCH
        //			RVLHTRANSFMX(pHypothesis->R_CHAL, pHypothesis->P_CHAL, TMS_CHAL);

        //			RMSE_CHAL = pShapeInstanceDetection->RMSE(pHypothesis->iModel, TGT, TMS_CHAL);

        //			fprintf(fpRMSE, "%f\t%f\n", RMSE_CHAL, RMSE_);
        //		}
        //		else
        //		{
        //			strHypothesisEval = strFP;

        //			nFP_++;
        //	}
        //	}
        //	else
        //	{
        //		strHypothesisEval = strFP;

        //		RMSE_ = 1.0f;

        //		nFP_++;
        //	}

        //	fprintf(fpInterpretation, "%s: hypothesisID: %07d\tiModel: %02d\tiSS: %03d\tscore:%04f\tRMSE: %f\tCTI: %f\tMatchedPerc: %02f\tgnd:%04f\tCHDist:%04f\n",
        //		strHypothesisEval, pHypothesis->idx, pHypothesis->iModel, pHypothesis->pSuperSegment->ID, pHypothesis->score, RMSE_, pHypothesis->CTIcost, pHypothesis->matchedPtsPercentage, pHypothesis->gndPlaneDistance, pHypothesis->CHMatchingMetric);
        //}

        float bestRMSE;
        int iBestSMatch;

        int *sHypothesesMatched = new int[interpretation2.size()];
        float *sBestRMSE = new float[interpretation2.size()];

        memset(sHypothesesMatched, 0xff, interpretation2.size() * sizeof(int));
        memset(sBestRMSE, 1.0f, interpretation2.size() * sizeof(float));

        for (iGT = 0; iGT < pGTArray->n; iGT++)
        {
            pGT = pGTArray->Element + iGT;

            bestRMSE = 0.03f;
            iBestSMatch = -1;

            int nHypothesesDEBUG = interpretation2.size();

            for (i = 0; i < interpretation2.size(); i++)
            {
                pHypothesis = interpretation2[i];

                if (pGT->iModel == pHypothesis->iModel && sHypothesesMatched[i] == -1)
                {
                    RVLHTRANSFMX(pHypothesis->R, pHypothesis->P, TMS);
                    RVLCOPYMX3X3(pGT->R, RGT);
                    RVLCOPY3VECTOR(pGT->t, tGT);
                    RVLHTRANSFMX(RGT, tGT, TGT);

                    RMSE_ = pShapeInstanceDetection->RMSE(pHypothesis->iModel, TGT, TMS);

                    if (RMSE_ <= bestRMSE)
                    {
                        bestRMSE = sBestRMSE[i] = RMSE_;
                        iBestSMatch = i;
                    }
                }
            }

            if (iBestSMatch != -1)
            {
                sHypothesesMatched[iBestSMatch] = iGT;
                GTHypothesis[iGT].pHypothesis = interpretation2[iBestSMatch];
            }
        }

        for (i = 0; i < interpretation2.size(); i++)
        {
            pHypothesis = interpretation2[i];

            if (sHypothesesMatched[i] == -1)
            {
                strHypothesisEval = strFP;

                nFP_++;

                fprintf(fpFP, "%d\t%f\t%f\t%f\n", sceneID, pHypothesis->matchedPtsPercentage, pHypothesis->score, pHypothesis->CHMatchingMetric);
            }
            else
            {
                strHypothesisEval = strTP;

                nTP_++;

                fprintf(fpTP, "%d\t%f\t%f\t%f\n", sceneID, pHypothesis->matchedPtsPercentage, pHypothesis->score, pHypothesis->CHMatchingMetric);
            }

            RMSE_ = sBestRMSE[i];

            fprintf(fpInterpretation, "%s: hypothesisID: %07d\tiModel: %02d\tiSS: %03d\tscore:%04f\tRMSE: %f\tCTI: %f\tMatchedPerc: %02f\tgnd:%04f\tCHDist:%04f\n",
                    strHypothesisEval, pHypothesis->idx, pHypothesis->iModel, pHypothesis->pSuperSegment->ID, pHypothesis->score, RMSE_, pHypothesis->CTIcost, pHypothesis->matchedPtsPercentage, pHypothesis->gndPlaneDistance, pHypothesis->CHMatchingMetric);
        }

        for (iGT = 0; iGT < pGTArray->n; iGT++)
            if (GTHypothesis[iGT].pHypothesis == NULL)
            {
                fprintf(fpInterpretation, "FN: iModel: %02d\n", pGTArray->Element[iGT].iModel);

                nFN_++;
            }

        precision_ = (float)nTP_ / ((float)nTP_ + (float)nFP_);
        recall_ = (float)nTP_ / ((float)nTP_ + (float)nFN_);

        nTP += nTP_;
        nFP += nFP_;
        nFN += nFN_;

        float precision = (float)nTP / ((float)nTP + (float)nFP);
        float recall = (float)nTP / ((float)nTP + (float)nFN);
        float F1 = 2 * precision * recall / (precision + recall);

        fprintf(fpInterpretation, "\nTP: %d\nFP: %d\nFN: %d\nprecision: %f\nrecall: %f\n\n", nTP_, nFP_, nFN_, precision_, recall_);
        fprintf(fpInterpretation, "\nTP: %d\nFP: %d\nFN: %d\nprecision: %f\nrecall: %f\nF1: %f\n\n", nTP, nFP, nFN, precision, recall, F1);

        fclose(fpInterpretation);

        fclose(fpCHAL);

        fclose(fpRMSE);

        fclose(fpTP);

        fclose(fpFP);
    }

    // Save interpretation to a file.

    FILE *fpInterpretation = fopen((std::string(resultsFolder) + "\\interpretation.dat").data(), "a");

    for (i = 0; i < interpretation2.size(); i++)
    {
        pHypothesis = interpretation2[i];

        fprintf(fpInterpretation, "%d\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\n", 0, sceneID, pHypothesis->iModel, pHypothesis->P[0], pHypothesis->P[1], pHypothesis->P[2],
                pHypothesis->R[2], pHypothesis->R[5], pHypothesis->R[8]);
    }

    fclose(fpInterpretation);

    bool bSaveSuperSegmentHypothesisDataToFile = false;

    if (bSaveSuperSegmentHypothesisDataToFile && pShapeInstanceDetection->pECCVGT->GT.Element)
    {
        // Save CTI match cost for 0th TP hypotheses
        FILE *fpTPCTICost = fopen((std::string(resultsFolder) + "\\TPCTIcost.txt").data(), "a");
        FILE *fpFPCTICost = fopen((std::string(resultsFolder) + "\\FPCTIcost.txt").data(), "a");

        fprintf(fpTPCTICost, "scene %d\n\n", sceneID);
        fprintf(fpFPCTICost, "scene %d\n\n", sceneID);

        char strTP[] = "TP";
        char strFP[] = "FP";

        char *strHypothesisEval;
        double TMS[16];
        float RGT[9];
        float tGT[3];
        double TGT[16];
        float RMSE_;

        for (iSuperSegment = 0; iSuperSegment < superSegments.n; iSuperSegment++)
        {
            if (sortedSuperSegmentHypotheses[iSuperSegment].n > 0)
            {
                pHypothesis = sortedSuperSegmentHypotheses[iSuperSegment].Element[0].ptr;

                iGT = iGTModel[pHypothesis->iModel];

                if (iGT >= 0)
                {
                    pGT = pGTArray->Element + iGT;

                    RVLHTRANSFMX(pHypothesis->R, pHypothesis->P, TMS);
                    RVLCOPYMX3X3(pGT->R, RGT);
                    RVLCOPY3VECTOR(pGT->t, tGT);
                    RVLHTRANSFMX(RGT, tGT, TGT);

                    RMSE_ = pShapeInstanceDetection->RMSE(pHypothesis->iModel, TGT, TMS);

                    if (RMSE_ <= 0.03f)
                    {
                        strHypothesisEval = strTP;

                        GTHypothesis[iGT].pHypothesis = pHypothesis;

                        fprintf(fpTPCTICost, "%s: hypothesisID: %07d\tiModel: %02d\tiSS: %03d\tRMSE: %f\tCTI: %f\n", strHypothesisEval, pHypothesis->idx, pHypothesis->iModel, pHypothesis->pSuperSegment->ID, RMSE_, pHypothesis->CTIcost);
                    }
                    else
                    {
                        strHypothesisEval = strFP;

                        fprintf(fpFPCTICost, "%s: hypothesisID: %07d\tiModel: %02d\tiSS: %03d\tRMSE: %f\tCTI: %f\n", strHypothesisEval, pHypothesis->idx, pHypothesis->iModel, pHypothesis->pSuperSegment->ID, RMSE_, pHypothesis->CTIcost);
                    }
                }
                else
                {
                    strHypothesisEval = strFP;

                    RMSE_ = 1.0f;

                    fprintf(fpFPCTICost, "%s: hypothesisID: %07d\tiModel: %02d\tiSS: %03d\tRMSE: %f\tCTI: %f\n", strHypothesisEval, pHypothesis->idx, pHypothesis->iModel, pHypothesis->pSuperSegment->ID, RMSE_, pHypothesis->CTIcost);
                }
            }
        }

        for (iGT = 0; iGT < pGTArray->n; iGT++)
            if (GTHypothesis[iGT].pHypothesis == NULL)
                fprintf(fpTPCTICost, "FN: iModel: %02d\n", pGTArray->Element[iGT].iModel);

        fprintf(fpTPCTICost, "\n\n");
        fprintf(fpFPCTICost, "\n\n");

        fclose(fpTPCTICost);
        fclose(fpFPCTICost);
    }
    // END - Save CTI match cost for 0th TP hypotheses

    // Save number of hypotheses to a file.

    FILE *fpNoHypotheses = fopen((std::string(resultsFolder) + "\\noHyp.dat").data(), "a");

    fprintf(fpNoHypotheses, "%d\t%d\t%d\t%d\t%d\t%d\n", superSegments.n, nMatches, nQualifiedMatches, nLevel1Hypotheses, nHypothesesEvaluatedByIP, nICPFittings);

    fclose(fpNoHypotheses);

    // Determine neighborhood for ICP for every supersegment.

    // SuperSegmentNeighbourhood(pMesh);		// Needed for PCL ICP

    delete[] superSegmentHypothesisList;
    delete[] superSegmentHypothesesMem;

    // Identify the best fitting hypothesis for each GT model (Parameter adjustment).

    double *PMMem = NULL;
    bool *bActiveModel = NULL;

    if (pShapeInstanceDetection->pECCVGT->GT.Element)
    {
        printf("Identify the best fitting hypothesis for each GT model...");

        bActiveModel = new bool[nModels];

        memset(bActiveModel, 0, nModels * sizeof(bool));

        int maxnCHVertices = 0;

        int nVertices;

        for (iModel = 0; iModel < pShapeInstanceDetection->activeModels.n; iModel++)
        {
            modelID = pShapeInstanceDetection->activeModels.Element[iModel];

            bActiveModel[modelID] = true;

            nVertices = modelCH[modelID]->iValidVertices.n;

            if (nVertices > maxnCHVertices)
                maxnCHVertices = nVertices;
        }

        PMMem = new double[3 * 8 * pGTArray->n];

        Array2D<double> PM;
        PM.w = 3;
        PM.h = 8;

        Box<double> modelBBox;
        vtkSmartPointer<vtkPolyData> model;

        float *PMCHMem = new float[3 * maxnCHVertices];

        for (iGT = 0; iGT < pGTArray->n; iGT++)
        {
            pGT = pGTArray->Element + iGT;

            iGTModel[pGT->iModel] = iGT;

            GTHypothesis[iGT].pHypothesis = NULL;
            GTHypothesis[iGT].BBoxCost = 1.0f;
            GTHypothesis[iGT].CHCost = 1.0f;

            if (bActiveModel[pGT->iModel])
            {
                model = pShapeInstanceDetection->vtkModelDB[pGT->iModel];

                MESH::BoundingBox<double>(model, &modelBBox);

                PM.Element = PMMem + 3 * 8 * iGT;

                BoxVertices<double>(&modelBBox, PM.Element);
            }
        }

        // fpDebug = fopen("C:\\RVL\\Debug\\debug.txt", "w");

        bool bAllHypotheses = true;
        // bool bAllHypotheses = false;

        pHypothesis = hypothesisList.pFirst;

        double RHGT[9];
        double tHGT[3];
        double V3Tmp[3];
        float RHGT_[9];
        float tHGT_[3];
        float EBB, ECH;

        while (pHypothesis)
        {
            if ((pHypothesis->flags & RVLVN_HYPOTHESIS2_FLAG_LEVEL1) || bAllHypotheses)
            {
                // iGT = iGTModel[pHypothesis->iModel];

                for (iGT = 0; iGT < pGTArray->n; iGT++)
                {
                    if (iGT >= 0 && bActiveModel[pHypothesis->iModel])
                    {
                        // if (iGT == 2)
                        //	int debug = 0;

                        // if (pHypothesis->iModel == 18)
                        //{
                        //	if (pHypothesis->pSuperSegment->ID != 27)
                        //	{
                        //		pHypothesis = pHypothesis->pNext;

                        //		continue;
                        //	}
                        //}

                        pGT = pGTArray->Element + iGT;

                        PM.Element = PMMem + 3 * 8 * iGT;

                        EBB = pShapeInstanceDetection->CompareBoundingBoxPoses(pGT->R, pGT->t, pHypothesis->R, pHypothesis->P, PM, pHypothesis->iModel);

                        pHypothesis->EBB = EBB;

                        // if (pHypothesis->idx == 19200)
                        //{
                        //	printf("Hypothesis 0019200: EBB=%f\n", EBB);

                        //	fprintf(fpDebug, "Model 6 BB: ");

                        //	if (pGT->iModel == 6)
                        //	{
                        //		for (i = 0; i < 24; i++)
                        //			fprintf(fpDebug, "%f ", PM.Element[i]);
                        //	}

                        //	fprintf(fpDebug, "\n");
                        //}

                        // fprintf(fpDebug, "%08d ", pHypothesis->idx);

                        // for (i = 0; i < 9; i++)
                        //	fprintf(fpDebug, "%f ", pHypothesis->R[i]);

                        // for (i = 0; i < 3; i++)
                        //	fprintf(fpDebug, "%f ", pHypothesis->P[i]);

                        // fprintf(fpDebug, "\n");

                        // RVLCOPYMX3X3(RHGT, RHGT_);
                        // RVLCOPY3VECTOR(tHGT, tHGT_);

                        // ECH = VN_::ComparePosesUsingConvexHull(RHGT_, tHGT_, modelCH[pHypothesis->iModel], PMCHMem);
                        ECH = 1.0f;

                        if (EBB < GTHypothesis[iGT].BBoxCost)
                        // if (ECH < GTHypothesis[iGT].CHCost)
                        {
                            GTHypothesis[iGT].pHypothesis = pHypothesis;
                            GTHypothesis[iGT].BBoxCost = EBB;
                            GTHypothesis[iGT].CHCost = ECH;
                        }
                    }
                    else
                        pHypothesis->EBB = 1.0f;
                }
            }

            pHypothesis = pHypothesis->pNext;
        }

        delete[] PMCHMem;

        printf("completed.\n");

        // fclose(fpDebug);

        // Print sorted hypotheses to file - For debug purpose only!
        FILE *fpTPSortedHypotheses = fopen((std::string(resultsFolder) + "\\TPSortedHypotheses.txt").data(), "a");
        FILE *fpSortedHypotheses = fopen((std::string(resultsFolder) + "\\SortedHypotheses.txt").data(), "a");

        fprintf(fpTPSortedHypotheses, "iScene: %d\n", sceneID);
        fprintf(fpSortedHypotheses, "iScene: %d\n", sceneID);

        // float gndPlaneDist;

        for (iGT = 0; iGT < pGTArray->n; iGT++)
        {
            pGT = pGTArray->Element + iGT;

            fprintf(fpTPSortedHypotheses, "GT model: %d\n", pGT->iModel);

            for (iSuperSegment = 0; iSuperSegment < superSegments.n; iSuperSegment++)
            {
                if (iGT == 0)
                    fprintf(fpSortedHypotheses, "iSuperSegment: %d\n", iSuperSegment);

                for (int iHypothesis = 0; iHypothesis < sortedSuperSegmentHypotheses[iSuperSegment].n; iHypothesis++)
                {
                    pHypothesis = sortedSuperSegmentHypotheses[iSuperSegment].Element[iHypothesis].ptr;

                    // gndPlaneDist = PointSetToPlaneDistance(modelCH[pHypothesis->iModel]->NodeArray, pHypothesis->R, pHypothesis->P, NGnd, dGnd);

                    // pHypothesis->gndPlaneDistance = gndPlaneDist;

                    if (iGT == 0)
                        fprintf(
                            fpSortedHypotheses,
                            "ID: %06d\t\tiSS: %02d\trank: %03d\tiModel: %02d\tcost: %f\tEBB: %f\tgndDist: %.8f\tCH: %.8f\tmatchedPerc: %.2f\n",
                            pHypothesis->idx, iSuperSegment, iHypothesis, pHypothesis->iModel, pHypothesis->score, sqrt(pHypothesis->EBB), pHypothesis->gndPlaneDistance, pHypothesis->CHMatchingMetric, pHypothesis->matchedPtsPercentage);

                    if ((pGT->iModel == pHypothesis->iModel) && (pHypothesis->EBB < 0.01f))
                    {
                        fprintf(
                            fpTPSortedHypotheses,
                            "iScene: %02d\tGT model: %02d\thypothesisID: %06d\tiSuperSegment: %02d\trank: %03d\tcost: %.4f\tEBB: %.8f\tgndDist: %.8f\tCH: %.8f\tmatchedPerc: %.2f\t(best cost: %.4f\tpercent: %.4f)\n",
                            sceneID, pGT->iModel, pHypothesis->idx, iSuperSegment, iHypothesis, pHypothesis->score, sqrt(pHypothesis->EBB), pHypothesis->gndPlaneDistance, pHypothesis->CHMatchingMetric, pHypothesis->matchedPtsPercentage,
                            sortedSuperSegmentHypotheses[iSuperSegment].Element[0].ptr->score, pHypothesis->score / sortedSuperSegmentHypotheses[iSuperSegment].Element[0].ptr->score);

                        // if (iHypothesis == 0)
                        // printf("GT model %d has hypothesis with rank 0! (cost: %f\tEBB: %f)\n", pGT->iModel, pHypothesis->score, pHypothesis->EBB);
                    }
                }

                if (iGT == 0)
                    fprintf(fpSortedHypotheses, "----------------------------------------------------------------\n");
            }

            fprintf(fpTPSortedHypotheses, "----------------------------------------------------------------\n");
        }

        fprintf(fpTPSortedHypotheses, "****************************************************************\n");
        fprintf(fpSortedHypotheses, "****************************************************************\n");

        fclose(fpTPSortedHypotheses);
        fclose(fpSortedHypotheses);
        // END - Print sorted hypotheses to file

#ifdef NEVER
        // Print model sorted hypotheses - For debug purpose only!
        FILE *fpModelSortedHypotheses = fopen((std::string(resultsFolder) + "\\ModelSortedHypotheses.txt").data(), "a");

        fprintf(fpModelSortedHypotheses, "iScene: %d\n", sceneID);

        for (iModel = 0; iModel < nModels; iModel++)
        {
            for (int iHypothesis = 0; iHypothesis < sortedModelHypotheses[iModel].n; iHypothesis++)
            {
                pHypothesis = sortedModelHypotheses[iModel].Element[iHypothesis].ptr;

                fprintf(
                    fpModelSortedHypotheses,
                    "ID: %06d\t\tiSS: %02d\trank: %03d\tiModel: %02d\tcost: %f\tEBB: %f\tgndDist: %.8f\tmatchedPerc: %.2f\n",
                    pHypothesis->idx, iSuperSegment, iHypothesis, pHypothesis->iModel, pHypothesis->score, sqrt(pHypothesis->EBB), pHypothesis->gndPlaneDistance, pHypothesis->matchedPtsPercentage);
            }
        }

        fprintf(fpModelSortedHypotheses, "----------------------------------------------------------------\n");

        fclose(fpModelSortedHypotheses);
        // END Print model sorted hypotheses
#endif

        // Find hypothesis with min EBB within nBest SS hypotheses - For debug purpose only!
        FILE *fpSortedHypothesesMinEBB = fopen((std::string(resultsFolder) + "\\SortedHypothesesMinEBB.txt").data(), "a");

        int debugNBestHypotheses = 50;
        int nBest;

        for (iGT = 0; iGT < pGTArray->n; iGT++)
        {
            pGT = pGTArray->Element + iGT;

            GTHypothesis[iGT].BBoxCost_ = 1.0f;
            GTHypothesis[iGT].pBestEBBTopHypothesis = NULL;
            GTHypothesis[iGT].rank = -1;

            for (iSuperSegment = 0; iSuperSegment < superSegments.n; iSuperSegment++)
            {
                nBest = RVLMIN(debugNBestHypotheses, sortedSuperSegmentHypotheses[iSuperSegment].n);

                for (int iHypothesis = 0; iHypothesis < nBest; iHypothesis++)
                {
                    pHypothesis = sortedSuperSegmentHypotheses[iSuperSegment].Element[iHypothesis].ptr;

                    if (pGT->iModel == pHypothesis->iModel)
                    {
                        if (pHypothesis->EBB < GTHypothesis[iGT].BBoxCost_)
                        {
                            GTHypothesis[iGT].BBoxCost_ = pHypothesis->EBB;
                            GTHypothesis[iGT].pBestEBBTopHypothesis = pHypothesis;
                            GTHypothesis[iGT].rank = iHypothesis;
                        }
                    }
                }
            }
        }

        // Print hypotheses with min EBB to file
        for (iGT = 0; iGT < pGTArray->n; iGT++)
        {
            pGT = pGTArray->Element + iGT;

            pHypothesis = GTHypothesis[iGT].pBestEBBTopHypothesis;

            if (pHypothesis)
                fprintf(
                    fpSortedHypothesesMinEBB,
                    "iScene: %02d\tGT model: %02d\thypothesisID: %06d\tiSuperSegment: %02d\trank: %03d\tcost: %.4f\tEBB: %.8f\tgndDist: %.8f\t(best cost: %.4f\tpercent: %.4f\tgndDist: %.8f iModel: %02d)\n",
                    sceneID, pGT->iModel, pHypothesis->idx, pHypothesis->pSuperSegment->ID, GTHypothesis[iGT].rank, pHypothesis->score, sqrt(pHypothesis->EBB), pHypothesis->gndPlaneDistance,
                    sortedSuperSegmentHypotheses[pHypothesis->pSuperSegment->ID].Element[0].ptr->score, pHypothesis->score / sortedSuperSegmentHypotheses[pHypothesis->pSuperSegment->ID].Element[0].ptr->score,
                    sortedSuperSegmentHypotheses[pHypothesis->pSuperSegment->ID].Element[0].ptr->gndPlaneDistance, sortedSuperSegmentHypotheses[pHypothesis->pSuperSegment->ID].Element[0].ptr->iModel);
        }

        fprintf(fpSortedHypothesesMinEBB, "----------------------------------------------------------------\n");

        fclose(fpSortedHypothesesMinEBB);
        // END - Find hypothesis with min EBB within nBest SS hypotheses

        //// Perform ICP fitting and compute RMSE for the best fitting hypothesis of each GT model (Parameter adjustment).

        // int maxnGTHypothesisSearchIterations = 5;

        // bool bRMSEOK;
        // int k;

        // for (iGT = 0; iGT < pGTArray->n; iGT++)
        //{
        //	pGT = pGTArray->Element + iGT;

        //	if (GTHypothesis[iGT].pHypothesis)
        //	{
        //		k = 0;

        //		do
        //		{
        //			pHypothesis = GTHypothesis[iGT].pHypothesis;

        //			ICP(ICPFunction, ICPVariant, pShapeInstanceDetection->vtkModelDB[pHypothesis->iModel], pHypothesis->R, pHypothesis->P,
        //				segmentN_PD.at(pHypothesis->pSuperSegment->ID), RMS, tMS);

        //			RVLHTRANSFMX(RMS, tMS, TMS);
        //			RVLCOPYMX3X3(pGT->R, RGT);
        //			RVLCOPY3VECTOR(pGT->t, tGT);
        //			RVLHTRANSFMX(RGT, tGT, TGT);

        //			RMSE_ = pShapeInstanceDetection->RMSE(pHypothesis->iModel, TGT, TMS);

        //			if (!(bRMSEOK = (RMSE_ <= 0.025f)))
        //			{
        //				GTHypothesis[iGT].pHypothesis = NULL;
        //				GTHypothesis[iGT].BBoxCost = 1.0f;

        //				pHypothesis->pSuperSegment->flags |= RVLVN_SUPERSEGMENT_FLAG_REJECTED;

        //				PM.Element = PMMem + 3 * 8 * iGT;

        //				pHypothesis = hypothesisList.pFirst;

        //				while (pHypothesis)
        //				{
        //					if ((pHypothesis->flags & RVLVN_HYPOTHESIS2_FLAG_LEVEL1) || bAllHypotheses)
        //					{
        //						if (!(pHypothesis->pSuperSegment->flags & RVLVN_SUPERSEGMENT_FLAG_REJECTED) && pHypothesis->iModel == pGT->iModel)
        //						{
        //							RVLCOMPTRANSF3DWITHINV(pGT->R, pGT->t, pHypothesis->R, pHypothesis->P, RHGT, tHGT, V3Tmp);

        //							EBB = pShapeInstanceDetection->CompareBoundingBoxPoses(RHGT, tHGT, PM);

        //							RVLCOPYMX3X3(RHGT, RHGT_);
        //							RVLCOPY3VECTOR(tHGT, tHGT_);

        //							ECH = VN_::ComparePosesUsingConvexHull(RHGT_, tHGT_, modelCH[pHypothesis->iModel], PMCHMem);

        //							//if (EBB < GTHypothesis[iGT].BBoxCost)
        //							if (ECH < GTHypothesis[iGT].CHCost)
        //							{
        //								GTHypothesis[iGT].pHypothesis = pHypothesis;
        //								GTHypothesis[iGT].BBoxCost = EBB;
        //								GTHypothesis[iGT].CHCost = ECH;
        //							}
        //						}
        //					}

        //					pHypothesis = pHypothesis->pNext;
        //				}

        //				if (GTHypothesis[iGT].pHypothesis == NULL)
        //					break;
        //			}

        //			k++;
        //		} while (!bRMSEOK && k < maxnGTHypothesisSearchIterations);
        //	}

        //	if (GTHypothesis[iGT].pHypothesis == NULL)
        //	{
        //		RMSE_ = 1.0f;
        //		RVLUNITMX3(RMS);
        //		RVLNULL3VECTOR(tMS);
        //	}

        //	GTHypothesis[iGT].RMSE = RMSE_;

        //	RVLCOPYMX3X3(RMS, GTHypothesis[iGT].R);
        //	RVLCOPY3VECTOR(tMS, GTHypothesis[iGT].t);

        //}

        // printf("completed.\n");

        // For each GT hypothesis find the hypothesis with the lowest eBB among the top nHypothesesLevel2 hypotheses generated from the same supersegment.

        VN_::Hypothesis2 *pTopHypothesis;

        for (iGT = 0; iGT < pGTArray->n; iGT++)
        {
            pGT = pGTArray->Element + iGT;

            pHypothesis = GTHypothesis[iGT].pHypothesis;

            if (pHypothesis == NULL)
                continue;

            superSegmentHypotheses_ = superSegmentHypotheses + pHypothesis->pSuperSegment->ID * nModels + pGT->iModel;

            PM.Element = PMMem + 3 * 8 * iGT;

            GTHypothesis[iGT].BBoxCost_ = 1.0f;
            GTHypothesis[iGT].pBestEBBTopHypothesis = NULL;

            for (i = 0; i < superSegmentHypotheses_->n; i++)
            {
                pTopHypothesis = superSegmentHypotheses_->Element[i].ptr;

                EBB = pShapeInstanceDetection->CompareBoundingBoxPoses(pGT->R, pGT->t, pTopHypothesis->R, pTopHypothesis->P, PM, pGT->iModel);

                if (EBB < GTHypothesis[iGT].BBoxCost_)
                {
                    GTHypothesis[iGT].BBoxCost_ = EBB;
                    GTHypothesis[iGT].pBestEBBTopHypothesis = pTopHypothesis;
                }
            }
        }

        //

        FILE *fpGTH = fopen((std::string(resultsFolder) + "\\GTH.txt").data(), "a");

        int level1, ID, rank;

        for (iGT = 0; iGT < pGTArray->n; iGT++)
        {
            pGT = pGTArray->Element + iGT;

            pHypothesis = GTHypothesis[iGT].pHypothesis;

            if (pHypothesis)
            {
                score = pHypothesis->score;
                level1 = (pHypothesis->flags & RVLVN_HYPOTHESIS2_FLAG_LEVEL1 ? 1 : 0);
                ID = pHypothesis->idx;
                iSuperSegment = pHypothesis->pSuperSegment->ID;
                rank = pHypothesis->rank;
            }
            else
            {
                GTHypothesis[iGT].RMSE = 1.0f;
                score = 1.0f;
                level1 = -1;
                ID = -1;
                iSuperSegment = -1;
                rank = -1;
            }

            // if (!HypothesisEvaluationCH(pHypothesis, eCH2avg, eCH2max))
            eCH2avg = eCH2max = 1.0f;

            fprintf(fpGTH, "%d\t%08d\t%d\t%d\t%f\t%f\t%f\t%d\t%f\t%f\t%f\t%f\n", sceneID, ID, iSuperSegment, pGT->iModel, GTHypothesis[iGT].RMSE, sqrt(GTHypothesis[iGT].BBoxCost), sqrt(GTHypothesis[iGT].BBoxCost_),
                    rank, score, GTHypothesis[iGT].CHCost, eCH2avg, eCH2max);

            // if (pGT->iModel == 18)
            //	for (i = 0; i < superSegmentHypotheses[pHypothesis->pSuperSegment->ID].n; i++)
            //		printf("%d %f\n", superSegmentHypotheses[pHypothesis->pSuperSegment->ID].Element[i].ptr->iModel, superSegmentHypotheses[pHypothesis->pSuperSegment->ID].Element[i].cost);
        }

        fclose(fpGTH);
    } // If the grount truth is available.

    delete[] D;
    D = NULL;
    delete[] NBuff;

    //// Visualization.

    if (visualizationData.bVisualizeSceneInterpretation)
    {
        // Display clusters.

        uchar SelectionColor[] = {0, 255, 0};

        RandomColors(SelectionColor, visualizationData.clusterColor, SClusters.n);

        InitDisplay(visualizationData.pVisualizer, pMesh, SelectionColor);

        // pSurfels->DisplayEdgeFeatures();

        // DisplayClusters();

        // pSurfels->DisplayVertices();

        // New visualization - 26.12.2019
        cv::Mat interpretationImage(camera.h, camera.w, CV_8UC3);

        uchar color[3] = {0, 255, 0};

        DisplayInterpretation(pMesh, color, interpretationImage, 2, true);

        cv::imwrite((std::string(resultsFolder) + "\\interpretation.tif").data(), interpretationImage);

        // Compute supersegment centroids.

        float *t;

        for (iSuperSegment = 0; iSuperSegment < superSegments.n; iSuperSegment++)
        {
            pSuperSegment = superSegments.Element[iSuperSegment];

            t = pSuperSegment->pose.t;

            RVLNULL3VECTOR(t);

            for (i = 0; i < pSuperSegment->iVertexArray.n; i++)
            {
                P = pSurfels->vertexArray.Element[pSuperSegment->iVertexArray.Element[i]]->P;

                RVLSUM3VECTORS(t, P, t);
            }

            RVLSCALE3VECTOR2(t, pSuperSegment->iVertexArray.n, t);
        }

        // Display GT hypotheses.

        double red[3] = {1.0f, 0.0f, 0.0f};
        double green[3] = {0.0f, 1.0f, 0.0f};
        double cyan[3] = {0.0f, 1.0f, 1.0f};

        bool bVisualizeTopSuperSegmentHypotheses = false;

        vtkSmartPointer<vtkActor> actor;

        /// Visualize top supersegment hypotheses.

        if (bVisualizeTopSuperSegmentHypotheses)
        {
            int *mask = new int[sampledUnitSphere.h];

            visualizationData.topHypothesisActors.Element = new vtkSmartPointer<vtkActor>[pGTArray->n];
            visualizationData.topHypothesisActors.n = 0;

            for (iGT = 0; iGT < pGTArray->n; iGT++)
            {
                pGT = pGTArray->Element + iGT;

                pHypothesis = GTHypothesis[iGT].pHypothesis;

                // if (iGT == 1)
                //	pHypothesis = superSegmentHypotheses[4].Element[0].ptr;

                if (pHypothesis)
                {
                    // actor = pShapeInstanceDetection->AddModelToVisualizer(visualizationData.pVisualizer, pShapeInstanceDetection->vtkModelDB[pGT->iModel], GTHypothesis[iGT].R, GTHypothesis[iGT].t, red);
                    // actor = pShapeInstanceDetection->AddModelToVisualizer(visualizationData.pVisualizer, pShapeInstanceDetection->vtkModelDB[pHypothesis->iModel], pHypothesis->R, pHypothesis->P, cyan);
                    actor = pShapeInstanceDetection->AddModelToVisualizer(visualizationData.pVisualizer, pShapeInstanceDetection->vtkModelDB[pGT->iModel], pGT->R, pGT->t, cyan);
                    // actor = pShapeInstanceDetection->AddModelToVisualizer(visualizationData.pVisualizer, pShapeInstanceDetection->vtkModelDB[pGT->iModel],
                    //	GTHypothesis[iGT].pBestEBBTopHypothesis->R, GTHypothesis[iGT].pBestEBBTopHypothesis->P, cyan, 2.0f);
                    visualizationData.topHypothesisActors.Element[visualizationData.topHypothesisActors.n++] = actor;

                    // float I3x3[9];
                    // RVLUNITMX3(I3x3);
                    // float V3DNull[3];
                    // RVLNULL3VECTOR(V3DNull);
                    // pShapeInstanceDetection->AddModelToVisualizer(visualizationData.pVisualizer, segmentN_PD.at(pBestHypothesis->pSuperSegment->ID), I3x3, V3DNull, color);

                    iSuperSegment = pHypothesis->pSuperSegment->ID;

                    pSuperSegment = superSegments.Element[iSuperSegment];

                    dS = DS + sampledUnitSphere.h * iSuperSegment;
                    bdS = bDS + sampledUnitSphere.h * iSuperSegment;

                    for (i = 0; i < sampledUnitSphere.h; i++)
                        mask[i] = (int)(bdS[i]);

                    float tSS[3];

                    RVLNULL3VECTOR(tSS);

                    // convexClustering.DisplayCTI(visualizationData.pVisualizer, sampledUnitSphere.Element, sampledUnitSphere.h, dS, pHypothesis->pSuperSegment->pose.R, tSS, mask);
                    // convexClustering.DisplayCTI(visualizationData.pVisualizer, sampledUnitSphere.Element, sampledUnitSphere.h, dS, pSuperSegment->pose.R, tSS, mask);

                    dM = DM + sampledUnitSphere.h * pHypothesis->iMCTI;

                    pMCTI = pShapeInstanceDetection->MCTISet.pCTI.Element[pHypothesis->iMCTI];

                    float RLMS[9];
                    float tLMS[3];

                    RVLCOMPTRANSF3D(pHypothesis->R, pHypothesis->P, pMCTI->R, tSS, RLMS, tLMS);

                    double modelColor[] = {0.0, 0.0, 1.0};

                    // convexClustering.DisplayCTI(visualizationData.pVisualizer, sampledUnitSphere.Element, sampledUnitSphere.h, dM, RLMS, tLMS, NULL, modelColor);
                }
            }

            delete[] mask;
            delete[] visualizationData.topHypothesisActors.Element;
        }
        else
        {
            /// Visualize interpretation.

            visualizationData.topHypothesisActors.Element = new vtkSmartPointer<vtkActor>[interpretation2.size()];
            visualizationData.topHypothesisActors.n = 0;

            int iObject;

            for (iObject = 0; iObject < interpretation2.size(); iObject++)
            {
                pHypothesis = interpretation2[iObject];

                if (pHypothesis)
                {
                    actor = pShapeInstanceDetection->AddModelToVisualizer(visualizationData.pVisualizer, pShapeInstanceDetection->vtkModelDB[pHypothesis->iModel],
                                                                          pHypothesis->R, pHypothesis->P, green, 2.0f);
                    visualizationData.topHypothesisActors.Element[visualizationData.topHypothesisActors.n++] = actor;
                }
            }

            delete[] visualizationData.topHypothesisActors.Element;
        }

        // visualizer.DisplayReferenceFrames(referenceFrames, 0.05);

        visualizationData.pVisualizer->Run();

        visualizationData.pVisualizer->renderer->RemoveAllViewProps();
    }

    ////

    printf("\n");

#ifdef NEVER
    for (iModel = 0; iModel < pShapeInstanceDetection->activeModels.n; iModel++)
    {
        modelID = pShapeInstanceDetection->activeModels.Element[iModel];

        for (iGT = 0; iGT < pGTArray->n; iGT++)
        {
            pGT = pGTArray->Element + iGT;

            if (pGT->iModel == modelID)
                break;
        }

        if (iGT >= pGTArray->n)
            continue;

        Box<double> modelBBox;

        vtkSmartPointer<vtkPolyData> model = pShapeInstanceDetection->vtkModelDB[modelID];

        InitBoundingBox<double>(&modelBBox, model->GetPoint(0));

        for (i = 1; i < model->GetNumberOfPoints(); i++)
            UpdateBoundingBox<double>(&modelBBox, model->GetPoint(i));

        Array2D<double> PM;
        double PMMem[3 * 8];
        PM.Element = PMMem;
        PM.w = 3;
        PM.h = 8;

        BoxVertices<double>(&modelBBox, PM.Element);

        float minE = 1.0f;
        float minE_ = 1.0f;

        VN_::Hypothesis2 *pBestHypothesis = NULL;
        VN_::Hypothesis2 *pBestHypothesis_ = NULL;

        for (iSuperSegment = 0; iSuperSegment < superSegments.n; iSuperSegment++)
        {
            for (i = 0; i < superSegmentHypotheses[iSuperSegment].n; i++)
            {
                pHypothesis = superSegmentHypotheses[iSuperSegment].Element[i].ptr;

                if (pHypothesis->iModel == modelID)
                {
                    RVLCOMPTRANSF3DWITHINV(pGT->R, pGT->t, pHypothesis->R, pHypothesis->P, RHGT, tHGT, V3Tmp);

                    EBB = pShapeInstanceDetection->CompareBoundingBoxPoses(RHGT, tHGT, PM);

                    if (E < minE)
                    {
                        minE = E;

                        pBestHypothesis = pHypothesis;
                    }

                    if (pHypothesis->score < minE_)
                    {
                        minE_ = pHypothesis->score;

                        pBestHypothesis_ = pHypothesis;
                    }
                }
            }
        }

        ICP(ICPFunction, ICPVariant, pShapeInstanceDetection->vtkModelDB[pBestHypothesis->iModel], pBestHypothesis->R, pBestHypothesis->P,
            segmentN_PD.at(pBestHypothesis->pSuperSegment->ID), RMS, tMS);

        double TMS[16];
        RVLHTRANSFMX(RMS, tMS, TMS);
        float RGT[9];
        RVLCOPYMX3X3(pGT->R, RGT);
        float tGT[3];
        RVLCOPY3VECTOR(pGT->t, tGT);
        double TGT[16];
        RVLHTRANSFMX(RGT, tGT, TGT);

        float RMSE_ = pShapeInstanceDetection->RMSE(pBestHypothesis->iModel, TGT, TMS);

        // Array<Pose3D> referenceFrames;

        // referenceFrames.Element = new Pose3D[nHypotheses];
        // referenceFrames.n = nHypotheses;

        // pBestHypothesis = pBestHypothesis_;

        pShapeInstanceDetection->AddModelToVisualizer(visualizationData.pVisualizer, pShapeInstanceDetection->vtkModelDB[pBestHypothesis->iModel], RMS, tMS, color);
        // pShapeInstanceDetection->AddModelToVisualizer(visualizationData.pVisualizer, pShapeInstanceDetection->vtkModelDB[pBestHypothesis->iModel], pBestHypothesis->R, pBestHypothesis->P, color);
        // pShapeInstanceDetection->AddModelToVisualizer(visualizationData.pVisualizer, pShapeInstanceDetection->vtkModelDB[pBestHypothesis->iModel], RGT, tGT, color);

        // float I3x3[9];
        // RVLUNITMX3(I3x3);
        // float V3DNull[3];
        // RVLNULL3VECTOR(V3DNull);
        // pShapeInstanceDetection->AddModelToVisualizer(visualizationData.pVisualizer, segmentN_PD.at(pBestHypothesis->pSuperSegment->ID), I3x3, V3DNull, color);

        iSuperSegment = pBestHypothesis->pSuperSegment->ID;

        dS = DS + sampledUnitSphere.h * iSuperSegment;
        bdS = bDS + sampledUnitSphere.h * iSuperSegment;

        int *mask = new int[sampledUnitSphere.h];

        for (i = 0; i < sampledUnitSphere.h; i++)
            mask[i] = (int)(bdS[i]);

        float tSS[3];

        RVLNULL3VECTOR(tSS);

        // convexClustering.DisplayCTI(visualizationData.pVisualizer, sampledUnitSphere.Element, sampledUnitSphere.h, dS, pBestHypothesis->pSuperSegment->pose.R, tSS, mask);

        dM = DM + sampledUnitSphere.h * pBestHypothesis->iMCTI;

        pMCTI = pShapeInstanceDetection->MCTISet.pCTI.Element[pBestHypothesis->iMCTI];

        float RLMS[9];
        float tLMS[3];

        RVLCOMPTRANSF3D(pBestHypothesis->R, pBestHypothesis->P, pMCTI->R, tSS, RLMS, tLMS);

        double modelColor[] = {0.0, 0.0, 1.0};

        // convexClustering.DisplayCTI(visualizationData.pVisualizer, sampledUnitSphere.Element, sampledUnitSphere.h, dM, RLMS, tLMS, NULL, modelColor);

        delete[] mask;
    } // for every active model.
#endif

    // Free memory.

    delete[] DS;
    delete[] bDS;
    RVL_DELETE_ARRAY(bActiveModel);
    RVL_DELETE_ARRAY(PMMem);
    RVL_DELETE_ARRAY(GTHypothesis);
    RVL_DELETE_ARRAY(iGTModel);
    // delete[] referenceFrames.Element;
    // delete[] clusterMapAll;
    delete[] superSegmentHypothesesMem2;
    delete[] superSegmentHypotheses;
    delete[] superSegmentHypothesesMem3;
    delete[] iObjectVertexArray.Element;
#ifdef NEVER // activate if hypotheses sorting by model is activated
    delete[] superSegmentHypothesesMem4;
#endif
    segmentMem.Free();
    FreeTmpMem();
}

bool VNClassifier::Classify(
    Mesh *pMesh,
    SURFEL::ObjectGraph *pObjects,
    Array<int> *piConvexObjects,
    Array<int> iSegmentArray,
    RECOG::VN_::FitData *fitData,
    QList<RECOG::VN_::Hypothesis> *pHypothesisList,
    float *d,
    uchar *bd,
    float *f,
    float *R,
    float *J,
    CRVLMem *pMem)
{
    if (pHypothesisList)
        RVLQLIST_INIT(pHypothesisList); // (ARP3D.TR3.9: Algorithm 2, line 5)

    float maxcsNGnd = 0.99f;

    // Consider ground plane or not.

    bool bGnd_ = (bGnd && bHorizontalBottomAssumption);

    // Make union of surfels and vertices of the segments in iSegmentArray (ARP3D.TR3.9: Algorithm 2, line 4).

    pObjects->UnionOfObjects(iSegmentArray, iSurfelArray, iVertexArray, bSurfelInArray, bVertexInArray);

    int support = pSurfels->SupportSize(iSurfelArray);

    // maxConvexityError <- maximum size of the intersection between the support set of the hypothesis and a convex segment.

    int maxConvexityError = 0;

    int i;

    if (convexityErrorCoeff > 1e-10)
    {
        if (piConvexObjects)
        {
            for (i = 0; i < iSurfelArray.n; i++)
                bSurfelInArray[iSurfelArray.Element[i]] = true;

            int intersection, convexityError;
            SURFEL::Object *pObject;
            QLIST::Index *piSurfel;

            for (i = 0; i < piConvexObjects->n; i++)
            {
                pObject = pObjects->objectArray.Element + piConvexObjects->Element[i];

                intersection = 0;

                piSurfel = pObject->surfelList.pFirst;

                while (piSurfel)
                {
                    if (bSurfelInArray[piSurfel->Idx])
                        intersection += pSurfels->NodeArray.Element[piSurfel->Idx].size;

                    piSurfel = piSurfel->pNext;
                }

                convexityError = (intersection > pObject->size / 2 ? pObject->size - intersection : intersection);

                if (convexityError > maxConvexityError)
                    maxConvexityError = convexityError;
            }

            for (i = 0; i < iSurfelArray.n; i++)
                bSurfelInArray[iSurfelArray.Element[i]] = false;
        }
    }

    // Copy vertices from pSurfels->vertexArray to convexHullVertices.

    pSurfels->GetVertices(iVertexArray, convexHullVertices);

    // Only for debugging purpose!

    // FILE *fpDebug = fopen("C:\\RVL\\Debug\\P.txt", "w");

    // PrintMatrix<float>(fpDebug, convexHullVertices.Element, convexHullVertices.h, 3);

    // fclose(fpDebug);

    //

    CRVLTimer timer;

    // Compute the approximate minimum bounding sphere of the input point set (ARP3D.TR3.9: Algorithm 2, a condition in line 1).

    timer.Start();

    float boundingSphereCenter[3];
    float boundingSphereRadius;

    BoundingSphere<float>(convexHullVertices, boundingSphereCenter, boundingSphereRadius);

    timer.Stop();

    double tBoundingSphere = timer.GetTime();

    // If the bounding sphere is too big, return false.

    if (boundingSphereRadius > maxBoundingSphereRadius)
        return false;

    // Create VN model and count outliers (ARP3D.TR3.9: Algorithm 2, lines 9, 10).

    VN *pVN = new VN;

    pVN->CreateEmpty();

    float I[9];

    RVLUNITMX3(I);

    float t[3];

    RVLNULL3VECTOR(t);

    Array2D<float> NCHArray;
    NCHArray.Element = NULL;
    float *dCH = NULL;

    Array2D<float> NConcavityArray;
    NConcavityArray.h = 0;
    float *dConcavity;

    //

    float *dS = NULL;

    Array<float> alphaArray;

    alphaArray.n = 0;

    Array<float> betaArray;

    betaArray.n = 0;

    bool bMetamodelConvex = true;
    bool bMetamodelConcave = false;

    int iFace;
    MESH::Face *pFace;
    int nOutliers;
    float *P, *P_;

    if (bDepthImage)
    {
        // Concavities.

        for (i = 0; i < iSegmentArray.n; i++)
            if (pObjects->objectArray.Element[iSegmentArray.Element[i]].flags & RVLPCSEGMENT_OBJECT_FLAG_CONCAVE)
                break;

        if (i < iSegmentArray.n) // if there are concave segments in iSegmentArray
        {
            bMetamodelConvex = false;

            // Create convex hull of the union of concave segments (ARP3D.TR3.9: Algorithm 1, line 6)

            Array<int> iConcaveSegmentArray;

            iConcaveSegmentArray.Element = new int[iSegmentArray.n];

            iConcaveSegmentArray.n = 0;

            for (i = 0; i < iSegmentArray.n; i++)
                if (pObjects->objectArray.Element[iSegmentArray.Element[i]].flags & RVLPCSEGMENT_OBJECT_FLAG_CONCAVE)
                    iConcaveSegmentArray.Element[iConcaveSegmentArray.n++] = iSegmentArray.Element[i];

            pObjects->UnionOfObjects(iConcaveSegmentArray, iConcavitySurfelArray, iConcavityVertexArray, bSurfelInArray, bVertexInArray);

            delete[] iConcaveSegmentArray.Element;

            for (i = 0; i < iConcavityVertexArray.n; i++)
            {
                P = pSurfels->vertexArray.Element[iConcavityVertexArray.Element[i]]->P;

                P_ = concavityVertices.Element + 3 * i;

                RVLCOPY3VECTOR(P, P_);
            }

            concavityVertices.h = iConcavityVertexArray.n;

            if (convexHull.ConvexHull(concavityVertices, &memConvexHull))
            {
                // Create complement of convexHull (ARP3D.TR3.9: Algorithm 1, line 7)

                convexHull.iVisibleFaces.n = 0;

                for (i = 0; i < convexHull.faces.n; i++)
                {
                    pFace = convexHull.faces.Element[i];

                    RVLNEGVECT3(pFace->N, pFace->N);

                    pFace->d = -pFace->d;

                    pFace->flags ^= RVLMESH_FACE_FLAG_VISIBLE;

                    if (pFace->flags & RVLMESH_FACE_FLAG_VISIBLE)
                        convexHull.iVisibleFaces.Element[convexHull.iVisibleFaces.n++] = i;
                }

                convexHull.VisibleSurface(NConcavityArray, dConcavity);

                pVN->AddModelCluster(0, RVLVN_CLUSTER_TYPE_CONCAVE, I, t, 0.5f, alphaArray, betaArray, NConcavityArray, pMem, 0.0f, NULL);
            }
        } // if there are concave segments in iSegmentArray

        // Compute the convex hull of the input point set (ARP3D.TR3.9: Algorithm 1, line 6).

        timer.Start();

        int nSamples;

        if (convexHull.ConvexHull(convexHullVertices, &memConvexHull))
        {
            timer.Stop();

            double tConvexHull = timer.GetTime();

            // Combine the convex and concave metamodel components into a polyhedron (ARP3D.TR3.9: Algorithm 1, line 9).

            timer.Start();

            convexHull.VisibleSurface(NCHArray, dCH);

            if (NConcavityArray.h > 0)
            {
                pVN->AddModelCluster(1, RVLVN_CLUSTER_TYPE_CONVEX, I, t, 0.5f, alphaArray, betaArray, NCHArray, pMem, 0.0f, NULL);

                pVN->AddOperation(2, 1, 0, 1, &memConvexHull);

                pVN->SetOutput(2);

                dS = new float[NCHArray.h + NConcavityArray.h];

                memcpy(dS, dConcavity, NConcavityArray.h * sizeof(float));
                memcpy(dS + NConcavityArray.h, dCH, NCHArray.h * sizeof(float));
            }
            else
            {
                pVN->AddModelCluster(0, RVLVN_CLUSTER_TYPE_CONVEX, I, t, 0.5f, alphaArray, betaArray, NCHArray, pMem, 0.0f, NULL);

                pVN->SetOutput(0);

                dS = dCH;
            }

            pVN->Create(pMem);

            timer.Stop();

            double tCreateVNFromConvexMesh = timer.GetTime();

            // Check whether the segment union iSegmentArray covers its entire convex hull - count outliers (ARP3D.TR3.9: Algorithm 2, line 10).

            float chamferDist;

            pVN->SceneFittingScore(pMesh, pSurfels, camera, iSurfelArray, iVertexArray, dS, fitParams.maxe, maxz,
                                   fitParams.cosSurfaceRayAngleThr, nSamples, chamferDist, nOutliers, visualizationData.bVisualizeSceneFitting);
        }
        else
            nOutliers = nSamples = 1;

        if (NConcavityArray.h > 0)
        {
            delete[] NConcavityArray.Element;
            delete[] dConcavity;
        }

        // If the percentage of outliers is > maxOutlierPerc, then no hypothesis is generated (ARP3D.TR3.9: Algorithm 2, line 13).

        if (nSamples == 0)
            nOutliers = nSamples = 1;

        if (100 * nOutliers / nSamples > maxOutlierPerc)
        {
            delete pVN;
            RVL_DELETE_ARRAY(dCH);
            if (NConcavityArray.h > 0)
                RVL_DELETE_ARRAY(dS);

            return false;
        }

        // All convex hull faces with normal similar to the opposite ground plane normal are considered as visible.

        if (bGnd_)
        {
            for (i = 0; i < convexHull.faces.n; i++)
            {
                pFace = convexHull.faces.Element[i];

                if (pFace->flags & RVLMESH_FACE_FLAG_VISIBLE)
                    continue;

                if (RVLDOTPRODUCT3(pFace->N, NGnd) > -maxcsNGnd)
                    continue;

                convexHull.iVisibleFaces.Element[convexHull.iVisibleFaces.n++] = i;
            }
        }

        // Convex hull faces defined by occlusion vertices only are considered as invisible.

        Array<int> iVisibleFaces;

        iVisibleFaces.Element = new int[convexHull.iVisibleFaces.n];

        iVisibleFaces.n = 0;

        MeshEdgePtr *pEdgePtr, *pEdgePtr0;
        int iCHVertex;
        SURFEL::Vertex *pVertex_;

        for (i = 0; i < convexHull.iVisibleFaces.n; i++)
        {
            iFace = convexHull.iVisibleFaces.Element[i];

            pFace = convexHull.faces.Element[iFace];

            pEdgePtr0 = pEdgePtr = pFace->pFirstEdgePtr;

            do
            {
                iCHVertex = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

                pVertex_ = pSurfels->vertexArray.Element[iVertexArray.Element[iCHVertex]];

                if (!(pVertex_->type & RVLSURFELVERTEX_TYPE_OCCLUSION))
                    break;

                pEdgePtr = pEdgePtr->pNext;

                if (pEdgePtr == NULL)
                    pEdgePtr = convexHull.NodeArray.Element[iCHVertex].EdgeList.pFirst;

                pEdgePtr = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_EDGE_PTR(pEdgePtr);
            } while (pEdgePtr != pEdgePtr0);

            if (!(pVertex_->type & RVLSURFELVERTEX_TYPE_OCCLUSION))
                iVisibleFaces.Element[iVisibleFaces.n++] = iFace;
        }

        if (iVisibleFaces.n == 0)
        {
            delete pVN;
            delete[] iVisibleFaces.Element;
            RVL_DELETE_ARRAY(dCH);
            if (NConcavityArray.h > 0)
                RVL_DELETE_ARRAY(dS);

            return false;
        }

        delete[] convexHull.iVisibleFaces.Element;

        convexHull.iVisibleFaces = iVisibleFaces;
    } // if (bDepthImage)
    else
    {
        for (i = 0; i < iSegmentArray.n; i++)
            if (pObjects->objectArray.Element[iSegmentArray.Element[i]].flags & RVLPCSEGMENT_OBJECT_FLAG_CONCAVE)
                bMetamodelConvex = false;
            else
                bMetamodelConcave = false;

        nOutliers = 0;

        // Compute the convex hull of the input point set.

        timer.Start();

        if (convexHull.ConvexHull(convexHullVertices, &memConvexHull))
        {
            timer.Stop();

            double tConvexHull = timer.GetTime();

            // Check whether the segment union iSegmentArray covers its entire convex hull.

            timer.Start();

            NCHArray.h = convexHull.iVisibleFaces.n = convexHull.faces.n;

            NCHArray.w = 3;

            NCHArray.Element = new float[NCHArray.w * NCHArray.h];

            dCH = new float[NCHArray.h];

            float *N;

            for (i = 0; i < convexHull.faces.n; i++)
            {
                convexHull.iVisibleFaces.Element[i] = i;

                convexHull.faces.Element[i]->flags |= RVLMESH_FACE_FLAG_VISIBLE;

                N = NCHArray.Element + 3 * i;

                pFace = convexHull.faces.Element[i];

                RVLCOPY3VECTOR(pFace->N, N);

                dCH[i] = pFace->d;
            }

            pVN->AddModelCluster(0, RVLVN_CLUSTER_TYPE_CONVEX, I, t, 0.5f, alphaArray, betaArray, NCHArray, pMem, 0.0f, NULL);

            pVN->SetOutput(0);

            dS = dCH;

            pVN->Create(pMem);

            timer.Stop();

            double tCreateVNFromConvexMesh = timer.GetTime();
        }
    } // if (!bDepthImage)

    RVL_DELETE_ARRAY(NCHArray.Element);

    QList<VN> *pVNList = &VNList;

    RVLQLIST_ADD_ENTRY(pVNList, pVN);

    // Only for debugging purpose!

    // if (convexHull.iVisibleFaces.n >= 19)
    //{
    //	pConvexHull->faces.Element[pConvexHull->iVisibleFaces.Element[6]]->Area = 1.0;
    //	pConvexHull->faces.Element[pConvexHull->iVisibleFaces.Element[7]]->Area = 1.0;
    //	pConvexHull->faces.Element[pConvexHull->iVisibleFaces.Element[10]]->Area = 1.0;
    //	pConvexHull->faces.Element[pConvexHull->iVisibleFaces.Element[11]]->Area = 1.0;
    //	pConvexHull->faces.Element[pConvexHull->iVisibleFaces.Element[13]]->Area = 1.0;
    // }

    ///

    Array<int> iSegmentArray_;
    RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, iSegmentArray.n, iSegmentArray_.Element);
    memcpy(iSegmentArray_.Element, iSegmentArray.Element, iSegmentArray.n * sizeof(int));
    iSegmentArray_.n = iSegmentArray.n;
    Array<int> iSurfelArray_;
    RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, iSurfelArray.n, iSurfelArray_.Element);
    memcpy(iSurfelArray_.Element, iSurfelArray.Element, iSurfelArray.n * sizeof(int));
    iSurfelArray_.n = iSurfelArray.n;
    Array<int> iVertexArray_;
    RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, iVertexArray.n, iVertexArray_.Element);
    memcpy(iVertexArray_.Element, iVertexArray.Element, iVertexArray.n * sizeof(int));
    iVertexArray_.n = iVertexArray.n;
    float *dS_;
    RVLMEM_ALLOC_STRUCT_ARRAY(pMem, float, pVN->featureArray.n, dS_);
    memcpy(dS_, dS, pVN->featureArray.n * sizeof(float));

    RVL_DELETE_ARRAY(dCH);

    if (NConcavityArray.h > 0)
        RVL_DELETE_ARRAY(dS);

    // Compute canonical orientation for convexHull.

    float RM0S[9];

    CanonicalOrientation(RM0S);

    // Compute descriptor.

    float *dH = new float[sampledUnitSphere.h];
    float *dH_ = new float[sampledUnitSphere.h];
    uchar *bdH = new uchar[sampledUnitSphere.h];
    uchar *bdH_ = new uchar[sampledUnitSphere.h];

    ComputeDescriptor(&convexHull, RM0S, dH, bdH, &iVertexArray);

    // Compute rotation cost Jacobian.

    ComputeRotationCostJacobian(&convexHull, RM0S, dH, bdH, J, &iVertexArray);

    // Visualize CTIs.

    Visualizer *pVisualizer = NULL;

    if (visualizationData.bVisualizeCTIs)
    {
        pVisualizer = new Visualizer;

        pVisualizer->Create();

        pVisualizer->SetMesh(pMesh);

        int *mask = new int[sampledUnitSphere.h];

        for (i = 0; i < sampledUnitSphere.h; i++)
            mask[i] = (int)(bdH[i]);

        convexClustering.DisplayCTI(pVisualizer, sampledUnitSphere.Element, sampledUnitSphere.h, dH, RM0S, t, mask);

        // float tCTIc_CTI[3];

        // vtkSmartPointer<vtkPolyData> polyData = GenerateCTIPrimitivePolydata_RW(sampledUnitSphere.Element, d__,
        //	sampledUnitSphere.h, false, mask, tCTIc_CTI);

        delete[] mask;

        // pVisualizer->AddMesh(polyData);

        pVisualizer->Run();

        pVisualizer->renderer->RemoveAllViewProps();
    }

    // Map the remaining descriptors to the reference descriptors.

    int nRollTotal = 4 * CTIDescriptorMapingData.nRoll;
    int nZ = CTIDescriptorMapingData.nPan * (CTIDescriptorMapingData.nTilt - 1) + 1;
    int nDescriptorsPerBlock = 6 * nRollTotal;
    int totalBlockDescriptorSize = nDescriptorsPerBlock * sampledUnitSphere.h;

    int j, k;
    float *f__;

    for (j = 0; j < nZ; j++)
    {
        f__ = f + j * totalBlockDescriptorSize;

        for (i = 0; i < totalBlockDescriptorSize; i++)
            f__[i] = f__[CTIDescriptorMapingData.iD[i]];
    }

    timer.Stop();

    double tFitCTI = timer.GetTime();

    // Classification.

    timer.Start();

    ///

    float totalArea = 0.0f;

    for (i = 0; i < convexHull.iVisibleFaces.n; i++)
        totalArea += convexHull.faces.Element[convexHull.iVisibleFaces.Element[i]]->Area;

    int nDescriptors = nZ * 6 * nRollTotal;

    float minE = -1.0f;

    bool bFirstClass = true;

    int iMostProbableClass = -1;

    Array<int> iAligned;

    iAligned.Element = new int[nDescriptors];

    Array<float *> secondAxisArray;

    secondAxisArray.Element = new float *[sampledUnitSphere.h];

    float *thirdAxis = new float[3 * sampledUnitSphere.h];

    int maxqSize = 0;

    int iClass;
    RECOG::ClassData *pClass;

    for (iClass = 0; iClass < classArray.n; iClass++)
    {
        pClass = classArray.Element + iClass;

        if (pClass->M.h > maxqSize)
            maxqSize = pClass->M.h;
    }

    float *q_ = new float[maxqSize];

    float *q0 = new float[maxqSize];

    memset(q0, 0, maxqSize * sizeof(float));

    bool *faceMask = new bool[sampledUnitSphere.h];

    int pq = 0;

    int id, idRef, iD, idEnd, iRoll_, nHypotheses, iHypothesis, nqs, iBlock, iR_;
    float correlation, maxCorrelation_, ELS, ELSV, E, w, bH, eqs, Eqs, ESP, z, zmin, wRelevant, maxf, f2;
    float *d__, *a, *a_, *AH, *q, *qs, *refAxis;
    uchar *bd__;
    float ROpt[9], RMS[9], W[3], PS[3], maxCorrelation[6], M3x3Tmp[9];
    bool bReal[3];
    int idx[3], iR[6];
    float C[9];
    VN_::Hypothesis *pHypothesis;
    // double eig[3];
    bool bMetamodelMatch, bFit;
    int *iD_, *intR;
    float *R__, *J__;

    for (iClass = 1; iClass < classArray.n; iClass++) // ECMR 2019
    // for (iClass = 0; iClass < classArray.n; iClass++)	// (ARP3D.TR3.9: Algorithm 2, line 16)
    {
        if (debug1 >= 0)
            if (iClass != debug1)
                continue;

        pClass = classArray.Element + iClass;

        bMetamodelMatch = true;

        if (bPrimitiveLayer)
        {
            if (pClass->bConvex && !bMetamodelConvex)
                bMetamodelMatch = false;

            if (pClass->bConcave && !bMetamodelConcave)
                bMetamodelMatch = false;
        }
        else
        {
            if (pClass->bConvex && !bMetamodelConvex)
                continue;

            if (pClass->bConcave && !bMetamodelConcave)
                continue;
        }

        // nHypotheses = 0;

        // for (j = 0; j < nDescriptors; j++, f_ += sampledUnitSphere.h)
        //{
        //	correlation = 0.0f;

        //	for (k = 0; k < pClass->faces.n; k++)
        //		correlation += f_[pClass->faces.Element[k]];

        //	if (correlation > 0.5f * maxCorrelation)
        //		nHypotheses++;
        //}

        memset(faceMask, 0, sampledUnitSphere.h * sizeof(bool));

        for (k = 0; k < pClass->faces.n; k++)
            faceMask[pClass->faces.Element[k]] = true;

#ifdef RVLVN_CLASSIFY_HYPOTHESIS_CTI_LOG
        FILE *fpDesc = fopen("desc.dat", "w");
#endif

        for (iHypothesis = 0; iHypothesis < alignment.CTIDescMap.nCTI; iHypothesis++)
        {
            iD_ = alignment.CTIDescMap.iD + sampledUnitSphere.h * iHypothesis;

            for (i = 0; i < sampledUnitSphere.h; i++)
            {
                dH_[i] = dH[iD_[i]];
                bdH_[i] = bdH[iD_[i]];
            }

            d__ = dH_;
            bd__ = bdH_;

            intR = alignment.CTIDescMap.R + 9 * iHypothesis;

            RVLMXMUL3X3(RM0S, intR, RMS);

            if (bPrimitiveLayer && iClass == 0)
            {
                R__ = primitiveLayerOutput.R + 9 * iHypothesis;

                RVLCOPYMX3X3(RMS, R__);

                J__ = primitiveLayerOutput.JR + 3 * iHypothesis;

                RVLMULMX3X3TVECT(intR, J, J__);

                if (J__[0] < 0.0f)
                    J__[0] = -J__[0];
                if (J__[1] < 0.0f)
                    J__[1] = -J__[1];
                if (J__[2] < 0.0f)
                    J__[2] = -J__[2];
            }

#ifdef RVLVN_CLASSIFY_HYPOTHESIS_CTI_LOG
            SaveDescriptor(fpDesc, dH_, bdH_, RMS, iHypothesis, 0);
#endif

            if (bMetamodelMatch)
                bFit = FitLS(d__, bd__, pClass, fitData + iClass, ELS);
            else
                bFit = false;

            if (bFit) // (ARP3D.TR3.9:	Algorithm 2, line 17;
                      //					fitData[iClass].q_ <- Eq. (3);
                      //					ELS <- the first termi in Eq. (4))
            {
                // Compute ELSV using Eq. (4) in ARP3D.TR3.9.

                ELSV = (bDepthImage ? ELS + invisibleComponentPenal * (float)(sampledUnitSphere.h / 2 - fitData[iClass].iV.n) : ELS);

                // Compute Eqs using Eq. (5) in ARP3D.TR3.9.

                nqs = pClass->M.h - 4;

                q = fitData[iClass].q_;

                qs = q + 4;

                Eqs = 0.0f;

                for (i = 0; i < pClass->nHull; i++)
                {
                    AH = pClass->A + nqs * i;

                    bH = q[3] * pClass->b[i];

                    RVLDOTPRODUCT(AH, qs, nqs, eqs, j);

                    eqs -= bH;

                    if (eqs > Eqs)
                        Eqs = eqs;
                }

                Eqs *= Eqs;

                // Compute E using Eq. (6) in ARP3D.TR3.9.

                E = (Eqs > ELSV ? Eqs : ELSV);

                // CTIMesh <- mesh representing CTI corresponding to the latent vector fitData[iClass].q_

                RVLMULMXTVECT(pClass->M.Element, fitData[iClass].q_, pClass->M.h, pClass->M.w, dH_, i, j, a);

#ifdef RVLVN_CLASSIFY_HYPOTHESIS_CTI_LOG
                // SaveDescriptor(fpDesc, dH_, bdH_, RMS, iHypothesis, 0);
#endif

                // printf("tCTIMesh=%lf\n", tCTIMesh);

                CRVLTimer timer2;

                timer2.Start();

                PSGM_::CreateCTIMesh(sampledUnitSphere, dH_, &CTIMesh, CTIMeshVertexMemSize, &memCTIMesh);

                timer2.Stop();

                double tCTIMesh = timer2.GetTime();

                // Compute the distance from the dominant planar surface and add it to the cost E.

                if (bGnd_)
                {
                    zmin = dGnd;

                    for (i = 0; i < CTIMesh.iValidVertices.n; i++)
                    {
                        P = CTIMesh.NodeArray.Element[CTIMesh.iValidVertices.Element[i]].P;

                        RVLMULMX3X3VECT(RMS, P, PS);

                        z = RVLDOTPRODUCT3(NGnd, PS);

                        if (z < zmin)
                            zmin = z;
                    }

                    zmin -= dGnd;

                    ESP = (zmin < 0.010f ? zmin : 0.0f);

                    ESP *= ESP;

                    // printf("iClass=%d E=%f ESP=%f\n", iClass, E, ESP);

                    E += (kZMin2 * ESP);
                }

                // minE <- minimum hypothesis cost
                // ROpt <- optimal orientation
                // q_ <- optimal latent vector
                // (ARP3D.TR3.9: Algorithm 2, lines 20-23)

                if (bFirstClass || E < minE)
                {
                    minE = E;

                    iMostProbableClass = iClass;

                    RVLCOPYMX3X3(RMS, ROpt);

                    memcpy(q_, fitData[iClass].q_, pClass->M.h * sizeof(float));

                    bFirstClass = false;
                }

                // FILE *fpDebug = fopen("C:\\RVL\\Debug\\P.txt", "w");

                // PrintMatrix<float>(fpDebug, pHypothesis->modelVertices.Element, pHypothesis->modelVertices.h, 3);

                // fclose(fpDebug);
            }
            else // If LS equations can't be solved
            {
                minE = E = invisibleComponentPenal * (float)(sampledUnitSphere.h);

                q = q0;
            }

            if (bPrimitiveLayer)
            {
                primitiveLayerOutput.y[iHypothesis * classArray.n + iClass] = E;

                memcpy(primitiveLayerOutput.q + iHypothesis * primitiveLayerOutput.mTotal + pq, q, pClass->M.h * sizeof(float));
            }
        } // for each hypothesis

#ifdef RVLVN_CLASSIFY_HYPOTHESIS_CTI_LOG
        fclose(fpDesc);
#endif
        pq += pClass->M.h;
    } // for every class

    // Create hypothesis (ARP3D.TR3.9: Algorithm 2, line 22)

    if (pHypothesisList)
    {
        RVLMEM_ALLOC_STRUCT(pMem, VN_::Hypothesis, pHypothesis);

        pHypothesis->pVN = pVN;
        pHypothesis->iSegmentArray = iSegmentArray_;
        pHypothesis->iSurfelArray = iSurfelArray_;
        pHypothesis->iVertexArray = iVertexArray_;
        pHypothesis->support = support;
        pHypothesis->d = dS_;
        pClass = classArray.Element + iMostProbableClass;
        if (bFirstClass)
        {
            pHypothesis->iClass = -1;
            pHypothesis->q = NULL;
            RVLUNITMX3(pHypothesis->R);
        }
        else
        {
            pHypothesis->iClass = iMostProbableClass;
            RVLMEM_ALLOC_STRUCT_ARRAY(pMem, float, pClass->M.h, pHypothesis->q);
            memcpy(pHypothesis->q, q_, pClass->M.h * sizeof(float));
            RVLCOPYMX3X3(ROpt, pHypothesis->R);
        }
        pHypothesis->shapeCost = minE;
        pHypothesis->convexityError = (float)maxConvexityError / (float)support;
        pHypothesis->nOutliers = nOutliers;
        pHypothesis->cost = pHypothesis->shapeCost + convexityErrorCoeff * pHypothesis->convexityError + outlierCost * (float)nOutliers;
        // RVLMEM_ALLOC_STRUCT_ARRAY(pMem, float, 3 * CTIMesh.iValidVertices.n, pHypothesis->modelVertices.Element);
        // pHypothesis->modelVertices.w = 3;
        // pHypothesis->modelVertices.h = CTIMesh.iValidVertices.n;
        // P_ = pHypothesis->modelVertices.Element;
        // for (i = 0; i < CTIMesh.iValidVertices.n; i++, P_ += 3)
        //{
        //	P = CTIMesh.NodeArray.Element[CTIMesh.iValidVertices.Element[i]].P;
        //	RVLCOPY3VECTOR(P, P_);
        // }

        RVLQLIST_ADD_ENTRY(pHypothesisList, pHypothesis);
    }

    timer.Stop();

    double tClassification = timer.GetTime();

    // printf("tClassification=%lf\n", tClassification);

    if (visualizationData.bVisualizeCTIs)
        delete pVisualizer;

    // Free memory.

    delete[] faceMask;
    delete[] dH;
    delete[] dH_;
    delete[] bdH;
    delete[] bdH_;
    delete[] iAligned.Element;
    delete[] secondAxisArray.Element;
    delete[] thirdAxis;
    delete[] q_;
    delete[] q0;

    //

    return true;
}

bool VNClassifier::Classify2(
    Mesh *pMesh,
    SURFEL::ObjectGraph *pObjects,
    Array<int> *piConvexObjects,
    Array<int> iSegmentArray,
    RECOG::VN_::FitData *fitData,
    QList<RECOG::VN_::Hypothesis> *pHypothesisList,
    float *d,
    uchar *bd,
    float *f,
    float *R,
    CRVLMem *pMem)
{
    float maxcsNGnd = 0.99f;

    // Make union of surfels and vertices of the segments in iSegmentArray.

    pObjects->UnionOfObjects(iSegmentArray, iSurfelArray, iVertexArray, bSurfelInArray, bVertexInArray);

    int support = pSurfels->SupportSize(iSurfelArray);

    // maxConvexityError <- maximum size of the intersection between the support set of the hypothesis and a convex segment.

    int maxConvexityError = 0;

    int i;

    if (piConvexObjects)
    {
        for (i = 0; i < iSurfelArray.n; i++)
            bSurfelInArray[iSurfelArray.Element[i]] = true;

        int intersection, convexityError;
        SURFEL::Object *pObject;
        QLIST::Index *piSurfel;

        for (i = 0; i < piConvexObjects->n; i++)
        {
            pObject = pObjects->objectArray.Element + piConvexObjects->Element[i];

            intersection = 0;

            piSurfel = pObject->surfelList.pFirst;

            while (piSurfel)
            {
                if (bSurfelInArray[piSurfel->Idx])
                    intersection += pSurfels->NodeArray.Element[piSurfel->Idx].size;

                piSurfel = piSurfel->pNext;
            }

            convexityError = (intersection > pObject->size / 2 ? pObject->size - intersection : intersection);

            if (convexityError > maxConvexityError)
                maxConvexityError = convexityError;
        }

        for (i = 0; i < iSurfelArray.n; i++)
            bSurfelInArray[iSurfelArray.Element[i]] = false;
    }

    // Copy vertices from pSurfels->vertexArray to convexHullVertices.

    float *P, *P_;

    for (i = 0; i < iVertexArray.n; i++)
    {
        P = pSurfels->vertexArray.Element[iVertexArray.Element[i]]->P;

        P_ = convexHullVertices.Element + 3 * i;

        RVLCOPY3VECTOR(P, P_);
    }

    convexHullVertices.h = iVertexArray.n;

    // Only for debugging purpose!

    // FILE *fpDebug = fopen("C:\\RVL\\Debug\\P.txt", "w");

    // PrintMatrix<float>(fpDebug, convexHullVertices.Element, convexHullVertices.h, 3);

    // fclose(fpDebug);

    //

    CRVLTimer timer;

    // Compute the approximate minimum bounding sphere of the input point set.

    timer.Start();

    float boundingSphereCenter[3];
    float boundingSphereRadius;

    BoundingSphere<float>(convexHullVertices, boundingSphereCenter, boundingSphereRadius);

    timer.Stop();

    double tBoundingSphere = timer.GetTime();

    // If the bounding sphere is to big, return false.

    if (boundingSphereRadius > maxBoundingSphereRadius)
        return false;

    // Create VN model.

    VN *pVN = new VN;

    pVN->CreateEmpty();

    float I[9];

    RVLUNITMX3(I);

    float t[3];

    RVLNULL3VECTOR(t);

    Array2D<float> NCHArray;
    NCHArray.Element = NULL;
    float *dCH = NULL;

    Array2D<float> NConcavityArray;
    NConcavityArray.h = 0;
    float *dConcavity;

    //

    float *dS = NULL;

    Array<float> alphaArray;

    alphaArray.n = 0;

    Array<float> betaArray;

    betaArray.n = 0;

    int iFace;
    MESH::Face *pFace;

    if (bDepthImage)
    {
        // Concavities.

        for (i = 0; i < iSegmentArray.n; i++)
            if (pObjects->objectArray.Element[iSegmentArray.Element[i]].flags & RVLPCSEGMENT_OBJECT_FLAG_CONCAVE)
                break;

        if (i < iSegmentArray.n) // if there are concave segments in iSegmentArray
        {
            Array<int> iConcaveSegmentArray;

            iConcaveSegmentArray.Element = new int[iSegmentArray.n];

            iConcaveSegmentArray.n = 0;

            for (i = 0; i < iSegmentArray.n; i++)
                if (pObjects->objectArray.Element[iSegmentArray.Element[i]].flags & RVLPCSEGMENT_OBJECT_FLAG_CONCAVE)
                    iConcaveSegmentArray.Element[iConcaveSegmentArray.n++] = iSegmentArray.Element[i];

            pObjects->UnionOfObjects(iConcaveSegmentArray, iConcavitySurfelArray, iConcavityVertexArray, bSurfelInArray, bVertexInArray);

            delete[] iConcaveSegmentArray.Element;

            for (i = 0; i < iConcavityVertexArray.n; i++)
            {
                P = pSurfels->vertexArray.Element[iConcavityVertexArray.Element[i]]->P;

                P_ = concavityVertices.Element + 3 * i;

                RVLCOPY3VECTOR(P, P_);
            }

            concavityVertices.h = iConcavityVertexArray.n;

            if (convexHull.ConvexHull(concavityVertices, &memConvexHull))
            {
                convexHull.iVisibleFaces.n = 0;

                for (i = 0; i < convexHull.faces.n; i++)
                {
                    pFace = convexHull.faces.Element[i];

                    RVLNEGVECT3(pFace->N, pFace->N);

                    pFace->d = -pFace->d;

                    pFace->flags ^= RVLMESH_FACE_FLAG_VISIBLE;

                    if (pFace->flags & RVLMESH_FACE_FLAG_VISIBLE)
                        convexHull.iVisibleFaces.Element[convexHull.iVisibleFaces.n++] = i;
                }

                convexHull.VisibleSurface(NConcavityArray, dConcavity);

                pVN->AddModelCluster(0, RVLVN_CLUSTER_TYPE_CONCAVE, I, t, 0.5f, alphaArray, betaArray, NConcavityArray, pMem, 0.0f, NULL);
            }
        }

        // Compute the convex hull of the input point set.

        timer.Start();

        int nSamples, nOutliers;

        if (convexHull.ConvexHull(convexHullVertices, &memConvexHull))
        {
            timer.Stop();

            double tConvexHull = timer.GetTime();

            // Check whether the segment union iSegmentArray covers its entire convex hull.

            timer.Start();

            // model.CreateFromConvexMesh(&convexHull, dCH, &memConvexHull);

            convexHull.VisibleSurface(NCHArray, dCH);

            if (NConcavityArray.h > 0)
            {
                pVN->AddModelCluster(1, RVLVN_CLUSTER_TYPE_CONVEX, I, t, 0.5f, alphaArray, betaArray, NCHArray, pMem, 0.0f, NULL);

                pVN->AddOperation(2, 1, 0, 1, &memConvexHull);

                pVN->SetOutput(2);

                dS = new float[NCHArray.h + NConcavityArray.h];

                memcpy(dS, dConcavity, NConcavityArray.h * sizeof(float));
                memcpy(dS + NConcavityArray.h, dCH, NCHArray.h * sizeof(float));
            }
            else
            {
                pVN->AddModelCluster(0, RVLVN_CLUSTER_TYPE_CONVEX, I, t, 0.5f, alphaArray, betaArray, NCHArray, pMem, 0.0f, NULL);

                pVN->SetOutput(0);

                dS = dCH;
            }

            pVN->Create(pMem);

            timer.Stop();

            double tCreateVNFromConvexMesh = timer.GetTime();

            float chamferDist;

            pVN->SceneFittingScore(pMesh, pSurfels, camera, iSurfelArray, iVertexArray, dS, fitParams.maxe, maxz,
                                   fitParams.cosSurfaceRayAngleThr, nSamples, chamferDist, nOutliers, visualizationData.bVisualizeSceneFitting);
        }
        else
            nOutliers = nSamples = 1;

        if (NConcavityArray.h > 0)
        {
            delete[] NConcavityArray.Element;
            delete[] dConcavity;
        }

        if (100 * nOutliers / nSamples > maxOutlierPerc)
            return false;

        // All convex hull faces with normal similar to the opposite ground plane normal are considered as visible.

        if (bGnd)
        {
            for (i = 0; i < convexHull.faces.n; i++)
            {
                pFace = convexHull.faces.Element[i];

                if (pFace->flags & RVLMESH_FACE_FLAG_VISIBLE)
                    continue;

                if (RVLDOTPRODUCT3(pFace->N, NGnd) > -maxcsNGnd)
                    continue;

                convexHull.iVisibleFaces.Element[convexHull.iVisibleFaces.n++] = i;
            }
        }

        // Convex hull faces defined by occlusion vertices only are considered as invisible.

        Array<int> iVisibleFaces;

        iVisibleFaces.Element = new int[convexHull.iVisibleFaces.n];

        iVisibleFaces.n = 0;

        MeshEdgePtr *pEdgePtr, *pEdgePtr0;
        int iCHVertex;
        SURFEL::Vertex *pVertex_;

        for (i = 0; i < convexHull.iVisibleFaces.n; i++)
        {
            iFace = convexHull.iVisibleFaces.Element[i];

            pFace = convexHull.faces.Element[iFace];

            pEdgePtr0 = pEdgePtr = pFace->pFirstEdgePtr;

            do
            {
                iCHVertex = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);

                pVertex_ = pSurfels->vertexArray.Element[iVertexArray.Element[iCHVertex]];

                if (!(pVertex_->type & RVLSURFELVERTEX_TYPE_OCCLUSION))
                    break;

                pEdgePtr = pEdgePtr->pNext;

                if (pEdgePtr == NULL)
                    pEdgePtr = convexHull.NodeArray.Element[iCHVertex].EdgeList.pFirst;

                pEdgePtr = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_EDGE_PTR(pEdgePtr);
            } while (pEdgePtr != pEdgePtr0);

            if (!(pVertex_->type & RVLSURFELVERTEX_TYPE_OCCLUSION))
                iVisibleFaces.Element[iVisibleFaces.n++] = iFace;
        }

        if (iVisibleFaces.n == 0)
        {
            delete pVN;
            delete[] iVisibleFaces.Element;
            RVL_DELETE_ARRAY(dCH);
            if (NConcavityArray.h > 0)
                RVL_DELETE_ARRAY(dS);

            return false;
        }

        delete[] convexHull.iVisibleFaces.Element;

        convexHull.iVisibleFaces = iVisibleFaces;
    } // if (bDepthImage)
    else
    {
        // Compute the convex hull of the input point set.

        timer.Start();

        if (convexHull.ConvexHull(convexHullVertices, &memConvexHull))
        {
            timer.Stop();

            double tConvexHull = timer.GetTime();

            // Check whether the segment union iSegmentArray covers its entire convex hull.

            timer.Start();

            NCHArray.h = convexHull.iVisibleFaces.n = convexHull.faces.n;

            NCHArray.w = 3;

            NCHArray.Element = new float[NCHArray.w * NCHArray.h];

            dCH = new float[NCHArray.h];

            float *N;

            for (i = 0; i < convexHull.faces.n; i++)
            {
                convexHull.iVisibleFaces.Element[i] = i;

                convexHull.faces.Element[i]->flags |= RVLMESH_FACE_FLAG_VISIBLE;

                N = NCHArray.Element + 3 * i;

                pFace = convexHull.faces.Element[i];

                RVLCOPY3VECTOR(pFace->N, N);

                dCH[i] = pFace->d;
            }

            pVN->AddModelCluster(0, RVLVN_CLUSTER_TYPE_CONVEX, I, t, 0.5f, alphaArray, betaArray, NCHArray, pMem, 0.0f, NULL);

            pVN->SetOutput(0);

            dS = dCH;

            pVN->Create(pMem);

            timer.Stop();

            double tCreateVNFromConvexMesh = timer.GetTime();
        }
    }

    RVL_DELETE_ARRAY(NCHArray.Element);

    QList<VN> *pVNList = &VNList;

    RVLQLIST_ADD_ENTRY(pVNList, pVN);

    // Only for debugging purpose!

    // if (convexHull.iVisibleFaces.n >= 19)
    //{
    //	pConvexHull->faces.Element[pConvexHull->iVisibleFaces.Element[6]]->Area = 1.0;
    //	pConvexHull->faces.Element[pConvexHull->iVisibleFaces.Element[7]]->Area = 1.0;
    //	pConvexHull->faces.Element[pConvexHull->iVisibleFaces.Element[10]]->Area = 1.0;
    //	pConvexHull->faces.Element[pConvexHull->iVisibleFaces.Element[11]]->Area = 1.0;
    //	pConvexHull->faces.Element[pConvexHull->iVisibleFaces.Element[13]]->Area = 1.0;
    // }

    ///

    Array<int> iSegmentArray_;
    RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, iSegmentArray.n, iSegmentArray_.Element);
    memcpy(iSegmentArray_.Element, iSegmentArray.Element, iSegmentArray.n * sizeof(int));
    iSegmentArray_.n = iSegmentArray.n;
    Array<int> iSurfelArray_;
    RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, iSurfelArray.n, iSurfelArray_.Element);
    memcpy(iSurfelArray_.Element, iSurfelArray.Element, iSurfelArray.n * sizeof(int));
    iSurfelArray_.n = iSurfelArray.n;
    Array<int> iVertexArray_;
    RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, iVertexArray.n, iVertexArray_.Element);
    memcpy(iVertexArray_.Element, iVertexArray.Element, iVertexArray.n * sizeof(int));
    iVertexArray_.n = iVertexArray.n;
    float *dS_;
    RVLMEM_ALLOC_STRUCT_ARRAY(pMem, float, pVN->featureArray.n, dS_);
    memcpy(dS_, dS, pVN->featureArray.n * sizeof(float));

    RVL_DELETE_ARRAY(dCH);

    if (NConcavityArray.h > 0)
        RVL_DELETE_ARRAY(dS);

    int *faceCorrespondence = new int[convexHull.iVisibleFaces.n];

    bool *faceMask = new bool[sampledUnitSphere.h];

    for (i = 0; i < sampledUnitSphere.h; i++)
        faceMask[i] = true;

    float U[9];

    InitNormalAlignment(convexHull.faces, convexHull.iVisibleFaces, U);

    float RXZ[9];
    float *X = RXZ;
    float *Y = RXZ + 3;
    float *Z = RXZ + 6;

    timer.Start();

    float dPan = 2.0f * PI / (float)(CTIDescriptorMapingData.nPan);
    float dTilt = 0.25f * PI / (float)(CTIDescriptorMapingData.nTilt);
    float dRoll = 0.5f * PI / (float)(CTIDescriptorMapingData.nRoll);

    int nRollTotal = 4 * CTIDescriptorMapingData.nRoll;
    int nZ = CTIDescriptorMapingData.nPan * (CTIDescriptorMapingData.nTilt - 1) + 1;

    int rollBlockSize = CTIDescriptorMapingData.nRoll * sampledUnitSphere.h;

    Point **pVertex = new Point *[rollBlockSize];

    int nDescriptorsPerBlock = 6 * nRollTotal;

    int totalBlockDescriptorSize = nDescriptorsPerBlock * sampledUnitSphere.h;

    memset(f, 0, nZ * totalBlockDescriptorSize * sizeof(float));

    float Z_[3];

    Z_[2] = 1.0f;

    float X_[3];

    X_[2] = 0.0f;

    float fmaxiTilt = (float)(CTIDescriptorMapingData.nTilt - 1);

    int iZ = 0;

    // Only for debugging purpose!

    // float *ZArchive = new float[3 * nZ];

    ///

    float *V3Tmp;
    int iPan, iTilt, iRoll, iRollBlockStart, nPan;
    float pan, tilt, roll, maxTilt, cPan, sPan, cTilt, sTilt, cRoll, sRoll, d0, d_, dmax, abscPan, abssPan, x, y, fTmp;
    float RRoll[9], R0[9], RSM[9];
    int j;
    float *f_;

    for (iTilt = 0; iTilt < CTIDescriptorMapingData.nTilt; iTilt++)
    {
        nPan = (iTilt > 0 ? CTIDescriptorMapingData.nPan : 1);

        for (iPan = 0; iPan < nPan; iPan++)
        {
            pan = (float)iPan * dPan;

            cPan = cos(pan);
            sPan = sin(pan);

            abscPan = RVLABS(cPan);
            abssPan = RVLABS(sPan);

            fTmp = (abscPan >= abssPan ? abscPan : abssPan);

            Z_[0] = cPan / fTmp;
            Z_[1] = sPan / fTmp;

            fTmp = sqrt(RVLDOTPRODUCT3(Z_, Z_));

            maxTilt = acos(1.0f / fTmp);

            tilt = maxTilt / fmaxiTilt * (float)iTilt;

            cTilt = cos(tilt);
            sTilt = sin(tilt);

            Z[0] = cPan * sTilt;
            Z[1] = sPan * sTilt;
            Z[2] = cTilt;

            // Only for debugging purpose!

            // float *ZArchive_ = ZArchive + 3 * iZ;

            // RVLCOPY3VECTOR(Z, ZArchive_);

            ///

            X_[0] = -Z[1];
            X_[1] = Z[0];

            fTmp = sqrt(RVLDOTPRODUCT3(X_, X_));

            if (fTmp >= 1e-10)
            {
                RVLSCALE3VECTOR2(X_, fTmp, X);
            }
            else
                RVLSET3VECTOR(X, 1.0f, 0.0f, 0.0f);

            RVLCROSSPRODUCT3(Z, X, Y);

            iRollBlockStart = iZ * totalBlockDescriptorSize;

            // if (iZ == 11)
            //	int debug = 0;

            for (iRoll = 0; iRoll < CTIDescriptorMapingData.nRoll; iRoll++)
            {
                roll = (float)iRoll * dRoll;

                cRoll = cos(roll);
                sRoll = sin(roll);

                RVLROTZ(cRoll, sRoll, RRoll);

                RVLMXMUL3X3(RRoll, RXZ, RSM);

                RVLCOPYMX3X3T(RSM, R0);

                FitCTI(&convexHull, U, R0, faceMask, 2, R + 9 * (iZ * CTIDescriptorMapingData.nRoll + iRoll), faceCorrespondence);

#ifdef RVLVN_CLASSIFY_CTI_VERSION1
                f_ = f + iRollBlockStart + iRoll * sampledUnitSphere.h;

                for (j = 0; j < convexHull.iVisibleFaces.n; j++)
                    f_[faceCorrespondence[j]] += convexHull.faces.Element[convexHull.iVisibleFaces.Element[j]]->Area;
#endif
            }

#ifdef RVLVN_CLASSIFY_CTI_VERSION0
            ComputeDescriptors(pConvexHull, R + 9 * iZ * CTIDescriptorMapingData.nRoll, d + iRollBlockStart, bd + iRollBlockStart, pVertex);
#endif

            iZ++;
        } // for (iPan = 0; iPan < nPan; iPan++)
    }     // for (iTilt = 0; iTilt < CTIDescriptorMapingData.nTilt; iTilt++)

    // Only for debugging purpose!

    // FILE *fpZArchive = fopen("C:\\RVL\\Debug\\Z.txt", "w");

    // PrintMatrix<float>(fpZArchive, ZArchive, nZ, 3);

    // fclose(fpZArchive);

    // delete[] ZArchive;

    ///

    // Map the remaining descriptors to the reference descriptors.

#ifdef RVLVN_CLASSIFY_CTI_VERSION0
    float *d__;

    for (j = 0; j < nZ; j++)
    {
        d__ = d + j * totalBlockDescriptorSize;

        for (i = 0; i < totalBlockDescriptorSize; i++)
            d__[i] = d__[CTIDescriptorMapingData.iD[i]];
    }

    timer.Stop();

    double tFitCTI = timer.GetTime();
#endif

#ifdef RVLVN_CLASSIFY_CTI_VERSION1
    float *f__;

    for (j = 0; j < nZ; j++)
    {
        f__ = f + j * totalBlockDescriptorSize;

        for (i = 0; i < totalBlockDescriptorSize; i++)
            f__[i] = f__[CTIDescriptorMapingData.iD[i]];
    }

    timer.Stop();

    double tFitCTI = timer.GetTime();

    // Classification.

    timer.Start();

    Visualizer *pVisualizer = NULL;

    if (visualizationData.bVisualizeCTIs)
    {
        pVisualizer = new Visualizer;

        pVisualizer->Create();
    }

    ///

    float totalArea = 0.0f;

    for (i = 0; i < convexHull.iVisibleFaces.n; i++)
        totalArea += convexHull.faces.Element[convexHull.iVisibleFaces.Element[i]]->Area;

    int nDescriptors = nZ * 6 * nRollTotal;

    float minE = -1.0f;

    bool bFirstClass = true;

    int iMostProbableClass = -1;

    RVLQLIST_INIT(pHypothesisList);

    float *dH = new float[sampledUnitSphere.h];

    Array<int> iAligned;

    iAligned.Element = new int[nDescriptors];

    Array<float *> secondAxisArray;

    secondAxisArray.Element = new float *[sampledUnitSphere.h];

    float *thirdAxis = new float[3 * sampledUnitSphere.h];

    Array<int> iFaceArray;

    iFaceArray.Element = new int[convexHull.iVisibleFaces.n];

    int iClass, k, id, idRef, iD, idEnd, iRoll_, nHypotheses, iHypothesis, nqs, iBlock, iR_, i_, j_, k_, iConvexTempleteElement;
    RECOG::ClassData *pClass;
    float correlation, maxCorrelation_, E, w, bH, eqs, Eqs, ESP, z, zmin, wRelevant, maxf, f2;
    float *R__, *d__, *RXZ_, *a, *a_, *AH, *q, *qs, *refAxis, *uS;
    uchar *bd__;
    float ROpt[9], R_[9], RMS[9], W[3], PS[3], maxCorrelation[6], M3x3Tmp[9], RMS_[9], uM[3];
    bool bReal[3];
    int idx[3], iR[6];
    float C[9];
    VN_::Hypothesis *pHypothesis;
    // double eig[3];

    for (iClass = 0; iClass < classArray.n; iClass++)
    {
        if (debug1 >= 0)
            if (iClass != debug1)
                continue;

        pClass = classArray.Element + iClass;

        memset(maxCorrelation, 0, 6 * sizeof(float));
        memset(iR, 0xff, 6 * sizeof(int));

        f_ = f;

        iAligned.n = 0;

        for (j = 0; j < nDescriptors; j++, f_ += sampledUnitSphere.h)
        {
            // if (j == 11 * 6 * 16)
            //	int debug = 0;

            // if (j == 10 * 6 * 16 + 2)
            //	int debug = 0;

            iBlock = (j / nRollTotal) % 6;

            if (pClass->bAxisAlignment)
            {
                correlation = 0.0f;

                for (k = 0; k < pClass->faces.n; k++)
                    correlation += f_[pClass->faces.Element[k]];
            }
            else
            {
                RVLNULLMX3X3(C);

                wRelevant = 0.0f;

                for (k = 0; k < pClass->faces.n; k++)
                {
                    i = pClass->faces.Element[k];

                    w = f_[i] / totalArea;

                    if (w < 1e-4)
                        continue;

                    wRelevant += w;

                    a = sampledUnitSphere.Element + 3 * i;

                    RVLSCALE3VECTOR(a, w, W);

                    RVLVECTCOV3(W, M3x3Tmp);

                    RVLSUMMX3X3(C, M3x3Tmp, C);
                }

                if (C[0] + C[4] + C[8] < 1e-8f)
                    continue;

                if (wRelevant < alignmentSupportThr)
                    continue;

                iAligned.Element[iAligned.n++] = j;

                // EigCov3<double>(C, eig, bReal);

                // if (!(bReal[0] && bReal[1] && bReal[2]))
                //	continue;

                // RVLSORT3ASCEND(eig, idx, fTmp);

                // correlation = eig[idx[1]] * eig[idx[2]];

                // Only for debugging purpose!

                // RVLCOMPLETESIMMX3(C);

                // FILE *fp = fopen("C:\\RVL\\Debug\\C.txt", "w");

                // PrintMatrix<float>(fp, C, 3, 3);

                // fclose(fp);

                //

                correlation = -C[2] * C[2] + C[0] * C[8] + C[4] * C[8] - C[1] * C[1] + C[0] * C[4] - C[5] * C[5];
            }

            if (correlation > maxCorrelation[iBlock])
            {
                maxCorrelation[iBlock] = correlation;

                iR[iBlock] = j;
            }
        } // for (j = 0; j < nDescriptors; j++, f_ += sampledUnitSphere.h)

        for (iBlock = 0; iBlock < 6; iBlock++)
            if (iR[iBlock] >= 0)
                break;

        if (iBlock >= 6)
        {
            maxCorrelation_ = 0.0f;

            for (j = 0; j < iAligned.n; j++)
            {
                f_ = f + iAligned.Element[j] * sampledUnitSphere.h;

                maxf = 0.0;

                for (k = 0; k < pClass->faces.n; k++)
                {
                    i = pClass->faces.Element[k];

                    f2 = f_[i];

                    if (f2 > maxf)
                    {
                        maxf = f2;

                        refAxis = sampledUnitSphere.Element + 3 * i;

                        w = f2 / totalArea;
                    }
                }

                RVLSCALE3VECTOR(refAxis, w, W);

                RVLVECTCOV3(W, C);

                secondAxisArray.n = 0;

                for (k = 0; k < pClass->faces.n; k++)
                {
                    i = pClass->faces.Element[k];

                    a = sampledUnitSphere.Element + 3 * i;

                    fTmp = RVLDOTPRODUCT3(refAxis, a);

                    if (RVLABS(fTmp) < 0.5f)
                    {
                        secondAxisArray.Element[secondAxisArray.n] = a;

                        a_ = thirdAxis + 3 * secondAxisArray.n;

                        RVLCROSSPRODUCT3(refAxis, a, a_);

                        secondAxisArray.n++;
                    }
                }

                for (i = 0; i < sampledUnitSphere.h; i++)
                {
                    w = f_[i] / totalArea;

                    if (w < 1e-4)
                        continue;

                    a = sampledUnitSphere.Element + 3 * i;

                    for (k = 0; k < secondAxisArray.n; k++)
                    {
                        a_ = secondAxisArray.Element[k];

                        fTmp = RVLDOTPRODUCT3(a, a_);

                        if (fTmp > 0.866f)
                        {
                            a_ = thirdAxis + 3 * k;

                            fTmp = RVLDOTPRODUCT3(a, a_);

                            if (RVLABS(fTmp) < 0.1)
                            {
                                RVLSCALE3VECTOR(a, w, W);

                                RVLVECTCOV3(W, M3x3Tmp);

                                RVLSUMMX3X3(C, M3x3Tmp, C);
                            }
                        }
                    }
                } // for (i = 0; i < sampledUnitSphere.h; i++)

                correlation = -C[2] * C[2] + C[0] * C[8] + C[4] * C[8] - C[1] * C[1] + C[0] * C[4] - C[5] * C[5];

                if (correlation > maxCorrelation_)
                {
                    maxCorrelation_ = correlation;

                    iR_ = iAligned.Element[j];

                    iBlock = (iR_ / nRollTotal) % 6;

                    iR[iBlock] = iR_;

                    maxCorrelation[iBlock] = maxCorrelation_;
                }
            } // for (j = 0; j < iAligned.n; j++)
        }     // if (iR < 0)

        // nHypotheses = 0;

        // for (j = 0; j < nDescriptors; j++, f_ += sampledUnitSphere.h)
        //{
        //	correlation = 0.0f;

        //	for (k = 0; k < pClass->faces.n; k++)
        //		correlation += f_[pClass->faces.Element[k]];

        //	if (correlation > 0.5f * maxCorrelation)
        //		nHypotheses++;
        //}

        memset(faceMask, 0, sampledUnitSphere.h * sizeof(bool));

        for (k = 0; k < pClass->faces.n; k++)
            faceMask[pClass->faces.Element[k]] = true;

        for (iHypothesis = 0; iHypothesis < pClass->iHypothesisBlocks.h; iHypothesis++)
        {
            maxCorrelation_ = 0.0f;

            iR_ = -1;

            for (j = 0; j < pClass->iHypothesisBlocks.w; j++)
            {
                iBlock = pClass->iHypothesisBlocks.Element[pClass->iHypothesisBlocks.w * iHypothesis + j];

                if (maxCorrelation[iBlock] > maxCorrelation_)
                {
                    iR_ = iR[iBlock];

                    maxCorrelation_ = maxCorrelation[iBlock];
                }
            }

            if (iR_ >= 0) // if there are valid alignment proposals
            {
                iZ = iR_ / nDescriptorsPerBlock;

                iRoll = iR_ % CTIDescriptorMapingData.nRoll;

                R__ = R + 9 * (iZ * CTIDescriptorMapingData.nRoll + iRoll);

                // idRef = iZ * totalBlockDescriptorSize + iRoll * sampledUnitSphere.h;

                // d__ = d + idRef;

                // bd__ = bd + idRef;

                // ComputeDescriptor(pConvexHull, R__, d__, bd__);

                // iD = iZ * totalBlockDescriptorSize;

                // d__ = d + iD;

                // bd__ = bd + iD;

                // id = iR * sampledUnitSphere.h - iD;

                // idEnd = id + sampledUnitSphere.h;

                // for (i = id; i < idEnd; i++)
                //{
                //	d__[i] = d__[CTIDescriptorMapingData.iD[i]];
                //	bd__[i] = bd__[CTIDescriptorMapingData.iD[i]];
                // }

                // d__ += id;
                // bd__ += id;

                // nHypotheses = (iClass < 2 ? 2 : 1);

                // for (iHypothesis = 0; iHypothesis < nHypotheses; iHypothesis++)
                {
                    iRoll_ = iR_ % nRollTotal;

                    roll = (float)(iRoll_ - iRoll) * dRoll;

                    cRoll = cos(roll);
                    sRoll = sin(roll);

                    RVLROTZ(cRoll, sRoll, RRoll);

                    // iBlock = (iHypothesis == 0 ? (iR_ / nRollTotal) % 6 : (iBlock + 3) % 6);

                    iBlock = (iR_ / nRollTotal) % 6;

                    RXZ_ = CTIDescriptorMapingData.R + 9 * iBlock;

                    RVLMXMUL3X3T2(RRoll, RXZ_, R_);

                    RVLMXMUL3X3(R__, R_, RMS_);

                    iFaceArray.n = 0;

                    for (j = 0; j < convexHull.iVisibleFaces.n; j++)
                    {
                        iFace = convexHull.iVisibleFaces.Element[j];

                        pFace = convexHull.faces.Element[iFace];

                        uS = pFace->N;

                        RVLGET_CLOSEST_CONVEX_TEMPLATE_ELEMENT(convexTemplateLUT, uS, RMS_, uM, i_, j_, k_, iConvexTempleteElement);

                        if (faceMask[iConvexTempleteElement])
                            iFaceArray.Element[iFaceArray.n++] = iFace;
                    }

                    InitNormalAlignment(convexHull.faces, iFaceArray, U);

                    FitCTI(&convexHull, U, RMS_, faceMask, 1, RMS, faceCorrespondence);

                    id = iR_ * sampledUnitSphere.h;

                    d__ = d + id;
                    bd__ = bd + id;

                    ComputeDescriptor(&convexHull, RMS, d__, bd__, &iVertexArray);

                    // Only for debugging purpose!

                    // float *d2 = new float[sampledUnitSphere.h];
                    // uchar *bd2 = new uchar[sampledUnitSphere.h];

                    // ComputeDescriptor(pConvexHull, RMS, d2, bd2);

                    // delete[] d2;
                    // delete[] bd2;

                    //

                    // Only for debugging purpose!

                    // FILE *fp = fopen("C:\\RVL\\Debug\\scenedescriptors.txt", "w");

                    // SaveDescriptor(fp, d__, bd__, R__, 0, 0);

                    // fclose(fp);

                    //

                    if (visualizationData.bVisualizeCTIs)
                    {
                        pVisualizer->SetMesh(pMesh);

                        int *mask = new int[sampledUnitSphere.h];

                        for (i = 0; i < sampledUnitSphere.h; i++)
                            mask[i] = (int)(bd__[i]);

                        convexClustering.DisplayCTI(pVisualizer, sampledUnitSphere.Element, sampledUnitSphere.h, d__, RMS, t, mask);

                        // float tCTIc_CTI[3];

                        // vtkSmartPointer<vtkPolyData> polyData = GenerateCTIPrimitivePolydata_RW(sampledUnitSphere.Element, d__,
                        //	sampledUnitSphere.h, false, mask, tCTIc_CTI);

                        delete[] mask;

                        // pVisualizer->AddMesh(polyData);

                        pVisualizer->Run();

                        pVisualizer->renderer->RemoveAllViewProps();
                    }

                    if (FitLS(d__, bd__, pClass, fitData + iClass, E))
                    {
                        if (bDepthImage)
                            E += (0.0001 * (sampledUnitSphere.h / 2 - fitData[iClass].iV.n));

                        if (iClass < 2)
                        {
                            nqs = pClass->M.h - 4;

                            q = fitData[iClass].q_;

                            qs = q + 4;

                            Eqs = 0.0f;

                            for (i = 0; i < pClass->nHull; i++)
                            {
                                AH = pClass->A + nqs * i;

                                bH = q[3] * pClass->b[i];

                                RVLDOTPRODUCT(AH, qs, nqs, eqs, j);

                                eqs -= bH;

                                if (eqs > Eqs)
                                    Eqs = eqs;
                            }

                            Eqs *= Eqs;

                            if (Eqs > E)
                                E = Eqs;
                        }

                        RVLMULMXTVECT(pClass->M.Element, fitData[iClass].q_, pClass->M.h, pClass->M.w, dH, i, j, a);

                        //

                        // printf("tCTIMesh=%lf\n", tCTIMesh);

                        CRVLTimer timer2;

                        timer2.Start();

                        PSGM_::CreateCTIMesh(sampledUnitSphere, dH, &CTIMesh, CTIMeshVertexMemSize, &memCTIMesh);

                        timer2.Stop();

                        double tCTIMesh = timer2.GetTime();

                        if (bGnd)
                        {
                            zmin = dGnd;

                            for (i = 0; i < CTIMesh.iValidVertices.n; i++)
                            {
                                P = CTIMesh.NodeArray.Element[CTIMesh.iValidVertices.Element[i]].P;

                                RVLMULMX3X3VECT(RMS, P, PS);

                                z = RVLDOTPRODUCT3(NGnd, PS);

                                if (z < zmin)
                                    zmin = z;
                            }

                            zmin -= dGnd;

                            ESP = (zmin < 0.010f ? zmin : 0.0f);

                            ESP *= ESP;

                            // printf("iClass=%d E=%f ESP=%f\n", iClass, E, ESP);

                            E += (kZMin2 * ESP);
                        }

                        RVLMEM_ALLOC_STRUCT(pMem, VN_::Hypothesis, pHypothesis);

                        pHypothesis->pVN = pVN;
                        pHypothesis->iSegmentArray = iSegmentArray_;
                        pHypothesis->iSurfelArray = iSurfelArray_;
                        pHypothesis->iVertexArray = iVertexArray_;
                        pHypothesis->support = support;
                        pHypothesis->iClass = iClass;
                        pHypothesis->d = dS_;
                        RVLMEM_ALLOC_STRUCT_ARRAY(pMem, float, pClass->M.h, pHypothesis->q);
                        memcpy(pHypothesis->q, fitData[iClass].q_, pClass->M.h * sizeof(float));
                        RVLCOPYMX3X3(RMS, pHypothesis->R);
                        pHypothesis->cost = E + convexityErrorCoeff * (float)maxConvexityError / (float)support;
                        RVLMEM_ALLOC_STRUCT_ARRAY(pMem, float, 3 * CTIMesh.iValidVertices.n, pHypothesis->modelVertices.Element);
                        pHypothesis->modelVertices.w = 3;
                        pHypothesis->modelVertices.h = CTIMesh.iValidVertices.n;
                        P_ = pHypothesis->modelVertices.Element;
                        for (i = 0; i < CTIMesh.iValidVertices.n; i++, P_ += 3)
                        {
                            P = CTIMesh.NodeArray.Element[CTIMesh.iValidVertices.Element[i]].P;
                            RVLCOPY3VECTOR(P, P_);
                        }

                        RVLQLIST_ADD_ENTRY(pHypothesisList, pHypothesis);

                        // FILE *fpDebug = fopen("C:\\RVL\\Debug\\P.txt", "w");

                        // PrintMatrix<float>(fpDebug, pHypothesis->modelVertices.Element, pHypothesis->modelVertices.h, 3);

                        // fclose(fpDebug);

                        // if (bFirstClass || E < minE)
                        //{
                        //	minE = E;

                        //	iMostProbableClass = iClass;

                        //	//RVLCOPYMX3X3(R__, ROpt);
                        //	RVLCOPYMX3X3(RMS, ROpt);

                        //	bFirstClass = false;
                        //}
                    }
                } // for each hypothesis
            }     // if there are valid alignment proposals
        }         // for each hypothesis
    }             // for every class

    timer.Stop();

    double tClassification = timer.GetTime();

    // printf("tClassification=%lf\n", tClassification);
#endif

    if (visualizationData.bVisualizeCTIs)
        delete pVisualizer;

    // Free memory.

    delete[] faceCorrespondence;
    delete[] faceMask;
    delete[] pVertex;
    delete[] dH;
    delete[] iAligned.Element;
    delete[] secondAxisArray.Element;
    delete[] thirdAxis;
    delete[] iFaceArray.Element;

    //

    return true;
}

float VNClassifier::MatchLatentVectors(
    float *q1,
    float s1,
    float *q2,
    float s2,
    int m,
    float &et,
    float &es,
    float &ea,
    float *E,
    int scale)
{
    float *qt1 = q1;
    float *qt2 = q2;
    float *qs1 = q1 + 3;
    float *qs2 = q2 + 3;

    int ms = m - 3;

    int i;

    RVLDIF3VECTORS(qt2, qt1, E);

    et = RVLDOTPRODUCT3(E, E) / varTranslation;

    // Method 1.

    // float *Es = E + 3;

    // RVLDOTPRODUCT(Es, Es, ms, es, i);

    // es /= varShape;

    // if (scale > 0)
    //{
    //	float s = (scale == 1 ? s1 * s1 : s2 * s2);

    //	et /= s;
    //	es /= s;
    //}

    // ea = 0.0f;

    // Method 2.

    float a = s1 * s2;

    RVLDOTPRODUCT(qs1, qs2, ms, es, i);

    es /= a;

    es = (1.0f - es * es) / varShape;

    et /= a;

    ea = s2 - s1;

    ea = ea * ea / a / varSize;

    float e = et + es + ea;

    return e;
}

RECOG::VN_::SuperSegment *VNClassifier::CreateSupersegment(
    Array<int> segments,
    float o)
{
    pObjects->UnionOfObjects(segments, iSurfelArray, iVertexArray, bSurfelInArray, bVertexInArray);

    // Copy vertices from pSurfels->vertexArray to convexHullVertices.

    pSurfels->GetVertices(iVertexArray, convexHullVertices);

    // Compute the approximate minimum bounding sphere of the input point set (ARP3D.TR3.9: Algorithm 2, a condition in line 1).

    float boundingSphereCenter[3];
    float boundingSphereRadius;

    BoundingSphere<float>(convexHullVertices, boundingSphereCenter, boundingSphereRadius);

    // If the bounding sphere is too big, return false.

    if (boundingSphereRadius > maxBoundingSphereRadius)
        return NULL;

    // Create convex hull of the segments in segments.

    if (!convexHull.ConvexHull(convexHullVertices, &memConvexHull))
        return NULL;

    // Hide faces defined by occlusion vertices.

    pSurfels->HideOcclustionFaces(&convexHull, iVertexArray);

    // Create supersegment.

    VN_::SuperSegment *pSuperSegment;

    RVLMEM_ALLOC_STRUCT(pMem, VN_::SuperSegment, pSuperSegment);
    pSuperSegment->segments.n = segments.n;
    RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, segments.n, pSuperSegment->segments.Element);
    memcpy(pSuperSegment->segments.Element, segments.Element, segments.n * sizeof(int));
    pSuperSegment->type = o;

    // Define reference frame.

    Array<Pose3D> referenceFrames;

    RECOG::ReferenceFrames(&convexHull, referenceFrames, kFaceClusterSizeThr, true);

    float *R = referenceFrames.Element[0].R;

    RVLCOPYMX3X3(R, pSuperSegment->pose.R);

    delete[] referenceFrames.Element;

    // Copy surfel and vertex data to pSuperSegment.

    RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, iSurfelArray.n, pSuperSegment->iSurfelArray.Element);

    memcpy(pSuperSegment->iSurfelArray.Element, iSurfelArray.Element, iSurfelArray.n * sizeof(int));
    pSuperSegment->iSurfelArray.n = iSurfelArray.n;

    RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, iVertexArray.n, pSuperSegment->iVertexArray.Element);

    memcpy(pSuperSegment->iVertexArray.Element, iVertexArray.Element, iVertexArray.n * sizeof(int));
    pSuperSegment->iVertexArray.n = iVertexArray.n;

    // Reset flags.

    pSuperSegment->flags = 0x00;

    // Free memory.

    return pSuperSegment;
}

int VNClassifier::CreateHypotheses(
    VN_::SuperSegment *pSuperSegment,
    float o,
    QList<VN_::Hypothesis> *pHypothesisList)
{
    pObjects->UnionOfObjects(pSuperSegment->segments, iSurfelArray, iVertexArray, bSurfelInArray, bVertexInArray);

    // Copy vertices from pSurfels->vertexArray to convexHullVertices.

    int i;
    float *P, *P_;

    for (i = 0; i < iVertexArray.n; i++)
    {
        P = pSurfels->vertexArray.Element[iVertexArray.Element[i]]->P;

        P_ = convexHullVertices.Element + 3 * i;

        RVLCOPY3VECTOR(P, P_);
    }

    convexHullVertices.h = iVertexArray.n;

    // Compute the approximate minimum bounding sphere of the input point set (ARP3D.TR3.9: Algorithm 2, a condition in line 1).

    float boundingSphereCenter[3];
    float boundingSphereRadius;

    BoundingSphere<float>(convexHullVertices, boundingSphereCenter, boundingSphereRadius);

    // If the bounding sphere is too big, return false.

    if (boundingSphereRadius > maxBoundingSphereRadius)
        return 0;

    // Create convex hull of the segments in segments.

    if (!convexHull.ConvexHull(convexHullVertices, &memConvexHull))
        return 0;

    // Define reference frames.

    Array<Pose3D> referenceFrames;

    RECOG::ReferenceFrames(&convexHull, referenceFrames, kFaceClusterSizeThr, true);

    // Create CTI from the highest ranked reference frame.

    // NHull is not used
    Array<SURFEL::NormalHullElement> NHull; // not used
    NHull.Element = NULL;
    NHull.n = 0;

    int nCTI = sampledUnitSphere.h;

    float *dS = new float[nCTI];
    uchar *bdS = new uchar[nCTI];
    float *NS = new float[nCTI * sampledUnitSphere.w];
    int *iV = new int[nCTI];

    float *R = referenceFrames.Element[0].R;

    ComputeDescriptor(iVertexArray, NHull, R, dS, bdS, NS, iV, o, true, false);

    // PSGM_::ModelInstance SCTI;

    // RVLCOPYMX3X3(R, SCTI.R);
    // RVLNULL3VECTOR(SCTI.t);

    // SCTI.modelInstance.Element = new PSGM_::ModelInstanceElement[sampledUnitSphere.h];

    // for (i = 0; i < nCTI; i++)
    //{
    //	SCTI.modelInstance.Element[i].d = dS[i];
    //	SCTI.modelInstance.Element[i].valid = (bdS[i] == 1);
    // }

    // Match scene CTI with the model CTIs.

    float *dM = new float[nCTI];
    uchar *bdM = new uchar[nCTI];
    float *e = new float[nCTI];

    float sigd12 = CTIMatchSigmad1 * CTIMatchSigmad1;
    float sigd22 = CTIMatchSigmad2 * CTIMatchSigmad2;

    float minE = (float)nCTI;

    int iBestMatch = -1;

    int iMCTI, nV;
    PSGM_::ModelInstance *pMCTI;
    float tMS[3];
    float E, e2, e2nrm;

    for (iMCTI = 0; iMCTI < pShapeInstanceDetection->MCTISet.pCTI.n; iMCTI++)
    {
        pMCTI = pShapeInstanceDetection->MCTISet.pCTI.Element[iMCTI];

        for (i = 0; i < nCTI; i++)
        {
            dM[i] = pMCTI->modelInstance.Element[i].d;
            bdM[i] = (uchar)(pMCTI->modelInstance.Element[i].valid);
        }

        FitLS(dM, bdM, dS, bdS, tMS, nV, e);

        E = 0.0f;

        for (i = 0; i < nCTI; i++)
        {
            e2 = e[i] * e[i];

            if (bdS[i])
                e2nrm = e2 / sigd12;
            else
            {
                if (e[i] < 0.0f)
                    e2nrm = e2 / sigd12;
                else
                    e2nrm = e2 / sigd22;
            }

            E += (e2nrm <= 1.0f ? e2nrm : 1.0f);
        }

        if (E < minE)
        {
            minE = E;

            iBestMatch = iMCTI;
        }
    }

    // Copy super segment data to pSuperSegment.

    RVLCOPYMX3X3(R, pSuperSegment->pose.R);

    RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, iSurfelArray.n, pSuperSegment->iSurfelArray.Element);

    memcpy(pSuperSegment->iSurfelArray.Element, iSurfelArray.Element, iSurfelArray.n * sizeof(int));
    pSuperSegment->iSurfelArray.n = iSurfelArray.n;

    RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, iVertexArray.n, pSuperSegment->iVertexArray.Element);

    memcpy(pSuperSegment->iVertexArray.Element, iVertexArray.Element, iVertexArray.n * sizeof(int));
    pSuperSegment->iVertexArray.n = iVertexArray.n;

    // Create new hypothesis.

    VN_::Hypothesis *pHypothesis;

    RVLMEM_ALLOC_STRUCT(pMem, VN_::Hypothesis, pHypothesis);

    RVLCOPYMX3X3(R, pHypothesis->R);

    delete[] referenceFrames.Element;

    pHypothesis->pSuperSegment = pSuperSegment;

    // Add hypothesis to pHypothesisList.

    RVLQLIST_ADD_ENTRY(pHypothesisList, pHypothesis);

    // Free memory.

    delete[] dS;
    delete[] bdS;
    delete[] dM;
    delete[] bdM;
    delete[] NS;
    delete[] iV;
    delete[] e;
    // delete[] SCTI.modelInstance.Element;

    return 1;
}

// Function VNClassifier::InterpretationCost requires VNClassifier::InitInterpretationCostComputation to be called before the first call
// and VNClassifier::DeallocateInterpretationCostComputationMem after the last call.

float VNClassifier::InterpretationCost(bool *X)
{
    memset(nSurfelCellHypotheses, 0, pObjects->surfelCells.n * sizeof(int));

    float cost = 0.0f;

    int i, j, iHypothesis;
    VN_::Hypothesis *pHypothesis;
    SURFEL::Cell *pSurfelCell;
    Array<int> *pObjectCells;

    for (iHypothesis = 0; iHypothesis < hypotheses.n; iHypothesis++)
    {
        if (!X[iHypothesis])
            continue;

        pHypothesis = hypotheses.Element[iHypothesis];

        cost += pHypothesis->cost;

        for (i = 0; i < pHypothesis->iSegmentArray.n; i++)
        {
            pObjectCells = pObjects->objectCells + pHypothesis->iSegmentArray.Element[i];

            for (j = 0; j < pObjectCells->n; j++)
                nSurfelCellHypotheses[pObjectCells->Element[j]]++;
        }
    }

    int iCell;
    int nHypotheses;

    for (iCell = 0; iCell < pObjects->surfelCells.n; iCell++)
    {
        nHypotheses = nSurfelCellHypotheses[iCell];

        if (nHypotheses == 0)
            cost += (unassignedPointCost * (float)(pObjects->surfelCells.Element[iCell].size));
        else if (nHypotheses > 1)
            cost += (multiplePointAssignmentCost * (float)((nHypotheses - 1) * pObjects->surfelCells.Element[iCell].size));
    }

    return cost;
}

void VNClassifier::InitInterpretationCostComputation()
{
    nSurfelCellHypotheses = new int[pObjects->surfelCells.n];
}

void VNClassifier::DeallocateInterpretationCostComputationMem()
{
    RVL_DELETE_ARRAY(nSurfelCellHypotheses);
}

void VNClassifier::Primitives(
    Mesh *pMesh,
    VNInstance *pVNInstance)
{
    bool bPrimitiveLog = true;

    // Segment mesh to surfels.

    pSurfels->Init(pMesh);

    pSurfelDetector->Init(pMesh, pSurfels, pMem);

    printf("Segmentation to surfels... ");

    double StartTime, ExecTime;

    if (pSurfelDetector->pTimer)
        StartTime = pSurfelDetector->pTimer->GetTime();

    pSurfelDetector->Segment(pMesh, pSurfels);

    if (pSurfelDetector->pTimer)
        ExecTime = pSurfelDetector->pTimer->GetTime() - StartTime;

    printf("completed.\n");
    printf("No. of surfels = %d\n", pSurfels->NodeArray.n);

    if (pSurfelDetector->pTimer)
        printf("Total segmentation time = %lf s\n", ExecTime);

    pSurfels->DetectVertices(pMesh);

    // Cluster surfels into convex and concave surfaces.

    Array<RECOG::SceneCluster> SClusters;

    bool bConcavity, bTorus;

    bConcavity = true;
    bTorus = false;

    int nSCClusters, nSUClusters;

    PSGM_::ConvexAndConcaveClusters(pMesh, pSurfels, pSurfelDetector, &convexClustering, &concaveClustering,
                                    SClusters, nSCClusters, nSUClusters, convexClustering.minClusterSize, NULL, maxnSCClusters, maxnSUClusters, bConcavity, (bTorus ? maxnSTClusters : 0),
                                    visualizationData.bVisualizeConvexClusters, visualizationData.bVisualizeConcaveClusters, visualizationData.bVisualizeSurfels);

    RECOG::CreateObjectGraphFromSceneClusters(SClusters, pObjects);

    pObjects->GetVertices();

    // Classify all clusters.

    maxBoundingSphereRadius = 10000.0f;
    bGnd = false;
    bDepthImage = false;

    descriptorMem.Clear();

    Init(pMesh);

    Array<int> iSegmentArray;

    iSegmentArray.n = 1;
    iSegmentArray.Element = new int[iSegmentArray.n];

    int maxm, m, iClass;

    if (pVNInstance)
    {
        pVNInstance->iPrimitiveClass = new int[pObjects->objectArray.n];

        pVNInstance->R = new float[9 * pObjects->objectArray.n];

        maxm = 0;

        for (iClass = 0; iClass < classArray.n; iClass++)
        {
            m = classArray.Element[iClass].M.h;

            if (m > maxm)
                maxm = m;
        }

        pVNInstance->q = new float[pObjects->objectArray.n * maxm];

        pVNInstance->nComponents = 0;
    }

    FILE *fpPrimitiveLayer = fopen((std::string(resultsFolder) + "\\primitiveLayer.dat").data(), "wb");

    float miny = 0.0f;

    int iObject, iClass_, iR_, iR;
    SURFEL::Object *pObject;
    ClassData *pClass;
    bool bFirst;
    float y;
    float *q, *q_;
    float J[3];

    for (iObject = 0; iObject < pObjects->objectArray.n; iObject++)
    {
        pObject = pObjects->objectArray.Element + iObject;

        if (pObject->flags & RVLPCSEGMENT_OBJECT_FLAG_CONCAVE)
            continue;

        iSegmentArray.Element[0] = iObject;

        if (!Classify(pMesh, pObjects, NULL, iSegmentArray, fitData, NULL, D, bD, f, R, J, pMem))
            continue;

        if (pVNInstance)
        {
            // (iClass_, iR_, q_) <- the most similar class instance

            q = primitiveLayerOutput.q;

            bFirst = true;

            for (iClass = 0; iClass < classArray.n; iClass++)
            {
                pClass = classArray.Element + iClass;

                for (iR = 0; iR < alignment.CTIDescMap.nCTI; iR++)
                {
                    y = primitiveLayerOutput.y[iClass * alignment.CTIDescMap.nCTI + iR];

                    if (bFirst || y < miny)
                    {
                        iClass_ = iClass;
                        iR_ = iR;
                        q_ = q;
                        miny = y;
                        bFirst = false;
                    }

                    q += pClass->M.h;
                }
            }

            // Copy (iClass_, iR_, q_) to pVNInstance

            pVNInstance->iPrimitiveClass[pVNInstance->nComponents] = iClass_;

            memcpy(pVNInstance->R + 9 * pVNInstance->nComponents, primitiveLayerOutput.R + 9 * iR_, 9 * sizeof(float));

            memcpy(pVNInstance->q + maxm * pVNInstance->nComponents, q_, classArray.Element[iClass_].M.h * sizeof(float));

            pVNInstance->nComponents++;
        }

        // Save primitive layer to a file.

        float w = (float)(pObject->size);

        for (iR = 0; iR < alignment.CTIDescMap.nCTI; iR++)
        {
            fwrite(&w, sizeof(float), 1, fpPrimitiveLayer);
            fwrite(primitiveLayerOutput.R + 9 * iR, sizeof(float), 9, fpPrimitiveLayer);
            fwrite(primitiveLayerOutput.JR + 3 * iR, sizeof(float), 3, fpPrimitiveLayer);
            fwrite(primitiveLayerOutput.y + classArray.n * iR, sizeof(float), classArray.n, fpPrimitiveLayer);
            fwrite(primitiveLayerOutput.q + primitiveLayerOutput.mTotal * iR, sizeof(float), primitiveLayerOutput.mTotal, fpPrimitiveLayer);
        }
    }

    fclose(fpPrimitiveLayer);

    int iPrimitive;

    if (pVNInstance)
    {
        // Save instance to a file.

        FILE *fp = fopen((std::string(resultsFolder) + "\\VNI.txt").data(), "w");

        int i;
        float *R;

        for (iPrimitive = 0; iPrimitive < pVNInstance->nComponents; iPrimitive++)
        {
            iClass = pVNInstance->iPrimitiveClass[iPrimitive];

            fprintf(fp, "%d\t", iClass);

            R = pVNInstance->R + 9 * iPrimitive;

            for (i = 0; i < 9; i++)
                fprintf(fp, "%f\t", R[i]);

            pClass = classArray.Element + iClass;

            m = classArray.Element[iClass].M.h;

            q = pVNInstance->q + maxm * iPrimitive;

            for (i = 0; i < m; i++)
                fprintf(fp, "%f\t", q[i]);

            for (; i < maxm; i++)
                fprintf(fp, "0.0\t");

            fprintf(fp, "\n");
        }

        fclose(fp);

        // Visualization

        Visualizer visualizer;

        visualizer.Create();

        visualizer.SetMesh(pMesh);

        float t[3];

        RVLNULL3VECTOR(t);

        float *a, *d;
        int j;

        pClass = classArray.Element;

        for (iPrimitive = 0; iPrimitive < pVNInstance->nComponents; iPrimitive++)
        // for (iPrimitive = 0; iPrimitive < 2; iPrimitive++)
        {
            d = new float[pClass->M.w];

            q = pVNInstance->q + maxm * iPrimitive;

            RVLMULMXTVECT(pClass->M.Element, q, pClass->M.h, pClass->M.w, d, i, j, a);

            convexClustering.DisplayCTI(&visualizer, sampledUnitSphere.Element, sampledUnitSphere.h, d, pVNInstance->R + 9 * iPrimitive, t);

            delete[] d;
        }

        visualizer.Run();
    }

    // bool bContinue = true;

    // uchar selectionColor[3] = { 0, 0, 255 };

    // uchar defaultColor[3] = { 255, 255, 0 };

    // RECOG::VN_::Hypothesis *pPrevHypothesis = NULL;

    // int iSurfel;
    // int j;
    // RECOG::ClassData *pClass;

    // while (bContinue)
    //{
    //	do
    //	{
    //		printf("Enter hypothesis ID: ");
    //		scanf("%d", &iHypothesis);
    //	} while (iHypothesis < -1 || iHypothesis > (int)(hypotheses.size()));

    //	if (iHypothesis == -1)
    //		break;

    //	if (pPrevHypothesis)
    //	{
    //		for (iSurfel = 0; iSurfel < pPrevHypothesis->iSurfelArray.n; iSurfel++)
    //			visualizer.PaintPointSet(&(pSurfels->NodeArray.Element[pPrevHypothesis->iSurfelArray.Element[iSurfel]].PtList),
    //			pMesh->pPolygonData, defaultColor);

    //		visualizer.renderer->RemoveAllViewProps();

    //		visualizer.SetMesh(pMesh);
    //	}

    //	pHypothesis = (RECOG::VN_::Hypothesis *)(hypotheses[iHypothesis]);

    //	if (pHypothesis)
    //	{
    //		for (iSurfel = 0; iSurfel < pHypothesis->iSurfelArray.n; iSurfel++)
    //			visualizer.PaintPointSet(&(pSurfels->NodeArray.Element[pHypothesis->iSurfelArray.Element[iSurfel]].PtList),
    //			pMesh->pPolygonData, selectionColor);

    //		pClass = classArray.Element + pHypothesis->iClass;

    //		d = new float[pClass->M.w];

    //		RVLMULMXTVECT(pClass->M.Element, pHypothesis->q, pClass->M.h, pClass->M.w, d, i, j, a);

    //		convexClustering.DisplayCTI(&visualizer, sampledUnitSphere.Element, sampledUnitSphere.h, d, pHypothesis->R, t);

    //		delete[] d;

    //		pPrevHypothesis = pHypothesis;
    //	}

    //	visualizer.Run();
    //}

    delete[] iSegmentArray.Element;
}

void VNClassifier::DetectPrimitives(
    Mesh *pMesh,
    RECOG::VN_::FitData *fitData,
    QList<RECOG::VN_::Hypothesis> *pHypothesisList,
    CRVLMem *pMem)
{
}

void VNClassifier::Interpretation()
{
    // Parameters

    int nIterations = simulatedAnnealingnIterationsPerHypothesis * hypotheses.n;
    int nPerturbances = 5;
    int perturbanceProbability = 50;

    // Initial interpretation by greedy search.

    GreedyInterpretation();

    bool *X = new bool[hypotheses.n];

    memset(X, 0, hypotheses.n * sizeof(bool));

    int i;

    for (i = 0; i < interpretation.n; i++)
        X[interpretation.Element[i]] = true;

    //

    bool *X_ = new bool[hypotheses.n];

    bool *XBest = new bool[hypotheses.n];

    int idxRange = 100 * hypotheses.n / perturbanceProbability;

    InitInterpretationCostComputation();

    // Only for debugging purpose!

    // memset(X_, 0, hypotheses.n * sizeof(bool));
    ////X[0] = X[1] = X[3] = X[4] = X[8] = X[11] = X[12] = X[17] = true;
    ////X[0] = X[1] = X[3] = X[14] = X[8] = X[11] = X[12] = X[17] = true;
    // X_[  0] = true;
    // X_[  1] = true;
    // X_[  2] = true;
    // X_[  3] = true;
    // X_[  5] = true;
    // X_[  7] = true;
    // X_[  8] = true;
    // X_[ 20] = true;
    // X_[ 21] = true;
    // X_[ 22] = true;
    // X_[ 28] = true;
    // X_[ 29] = true;
    // X_[ 37] = true;
    // X_[ 38] = true;
    // X_[ 43] = true;
    // X_[ 52] = true;
    // X_[ 72] = true;
    // X_[ 96] = true;
    // X_[101] = true;
    // X_[108] = true;

    // float cost1 = InterpretationCost(X_);

    // memset(X_, 0, hypotheses.n * sizeof(bool));
    // X_[0] = true;
    // X_[1] = true;
    // X_[2] = true;
    // X_[3] = true;
    // X_[7] = true;
    // X_[10] = true;
    // X_[20] = true;
    // X_[21] = true;
    // X_[22] = true;
    // X_[28] = true;
    // X_[29] = true;
    // X_[37] = true;
    // X_[38] = true;
    // X_[43] = true;
    // X_[52] = true;
    // X_[72] = true;
    // X_[96] = true;
    // X_[101] = true;
    // X_[108] = true;

    // float cost2 = InterpretationCost(X_);

    // memcpy(X, X_, hypotheses.n * sizeof(bool));

    // Interpretation using local search.

    memcpy(XBest, X, hypotheses.n * sizeof(bool));

    float minCost = InterpretationCost(X);

    int j, k, iHypothesis;
    float cost;

    for (k = 0; k < nIterations; k++)
    {
        for (i = 0; i < simulatedAnnealingnSamples; i++)
        {
            memcpy(X_, X, hypotheses.n * sizeof(bool));

            for (j = 0; j < nPerturbances; j++)
            {
                iHypothesis = rand() % idxRange;

                if (iHypothesis < hypotheses.n)
                    X_[iHypothesis] = !X_[iHypothesis];
            }

            cost = InterpretationCost(X_);

            if (cost < minCost)
            {
                minCost = cost;

                memcpy(XBest, X_, hypotheses.n * sizeof(bool));
            }
        }

        memcpy(X, XBest, hypotheses.n * sizeof(bool));
    }

    DeallocateInterpretationCostComputationMem();

    // Copy X to interpretation.

    interpretation.n = 0;

    for (iHypothesis = 0; iHypothesis < hypotheses.n; iHypothesis++)
        if (X[iHypothesis])
            interpretation.Element[interpretation.n++] = iHypothesis;

    // Free memory.

    delete[] X;
    delete[] X_;
    delete[] XBest;
}

void VNClassifier::GreedyInterpretation()
{
    // Sort hypotheses.

    RVL_DELETE_ARRAY(sortedHypotheses.Element);

    sortedHypotheses.Element = new SortIndex<float>[hypotheses.n];

    int i;

    for (i = 0; i < hypotheses.n; i++)
    {
        sortedHypotheses.Element[i].idx = i;
        sortedHypotheses.Element[i].cost = hypotheses.Element[i]->cost;
    }

    sortedHypotheses.n = hypotheses.n;

    BubbleSort<SortIndex<float>>(sortedHypotheses);

    // Hypothesis pruning.

    RVL_DELETE_ARRAY(interpretation.Element);

    interpretation.Element = new int[hypotheses.n];

    interpretation.n = 0;

    bool *bSurfelOccupied = new bool[pSurfels->NodeArray.n];

    memset(bSurfelOccupied, 0, pSurfels->NodeArray.n * sizeof(bool));

    int k, iHypothesis;
    RECOG::VN_::Hypothesis *pHypothesis;
    SURFEL::Object *pObject_;
    QLIST::Index *piElement;
    int occupied;

    for (i = 0; i < hypotheses.n; i++)
    {
        iHypothesis = sortedHypotheses.Element[i].idx;

        pHypothesis = hypotheses.Element[iHypothesis];

        occupied = 0;

        for (k = 0; k < pHypothesis->iSegmentArray.n; k++)
        {
            pObject_ = pObjects->objectArray.Element + pHypothesis->iSegmentArray.Element[k];

            piElement = pObject_->surfelList.pFirst;

            while (piElement)
            {
                if (bSurfelOccupied[piElement->Idx])
                    occupied += pSurfels->NodeArray.Element[piElement->Idx].size;

                piElement = piElement->pNext;
            }
        }

        if (occupied * 100 / pHypothesis->support > collisionPercThr)
            continue;

        for (k = 0; k < pHypothesis->iSegmentArray.n; k++)
        {
            pObject_ = pObjects->objectArray.Element + pHypothesis->iSegmentArray.Element[k];

            piElement = pObject_->surfelList.pFirst;

            while (piElement)
            {
                bSurfelOccupied[piElement->Idx] = true;

                piElement = piElement->pNext;
            }
        }

        interpretation.Element[interpretation.n++] = iHypothesis;
    }

    delete[] bSurfelOccupied;
}

// Move to Util.h

namespace RVL
{
    int Area(Rect<int> rect)
    {
        return (rect.maxx - rect.minx + 1) * (rect.maxy - rect.miny + 1);
    }
}

void VNClassifier::ImageSegmentation(
    Mesh *pMesh,
    Array2D<int> &objectMap)
{
    float ROIBorderSize = 10;

    Rect<int> cropWin;
    cropWin.minx = 0;
    cropWin.maxx = camera.w - 1;
    cropWin.miny = 0;
    cropWin.maxy = camera.h - 1;

    int nPix = camera.w * camera.h;

    objectMap.w = camera.w;
    objectMap.h = camera.h;

    objectMap.Element = new int[nPix];

    float *ez = new float[nPix];

    bool *bValidPt = new bool[nPix];

    int iPix = 0;

    int u, v;
    float s, z, fTmp, e;
    float r[3];
    Point *pPt;

    for (v = 0; v < camera.h; v++)
        for (u = 0; u < camera.w; u++, iPix++)
        {
            bValidPt[iPix] = false;

            objectMap.Element[iPix] = -2;

            ez[iPix] = 10.0f;

            // pPt = pMesh->NodeArray.Element + iPix;

            ////bValidPt[iPix] = (RVLDOTPRODUCT3(pPt->N, pPt->N) >= 1e-10);
            // r[0] = ((float)u - camera.uc) / camera.fu;
            // r[1] = ((float)v - camera.vc) / camera.fv;
            // r[2] = 1.0f;

            // fTmp = RVLDOTPRODUCT3(NGnd, r);

            // if (fTmp < -1e-10)
            //{
            //	s = dGnd / fTmp;

            //	z = s * r[2];

            //	if (z <= maxz)
            //	{
            //		objectMap.Element[iPix] = -1;

            //		if (bValidPt[iPix])
            //		{
            //			e = RVLDOTPRODUCT3(NGnd, pPt->P) - dGnd;

            //			ez[iPix] = RVLABS(e);
            //		}
            //		else
            //			ez[iPix] = z;
            //	}
            //}
        }

    int maxnVNFeatures = 0;

    int iHypothesis;
    VN_::Hypothesis *pHypothesis;

    for (iHypothesis = 0; iHypothesis < interpretation.n; iHypothesis++)
    {
        pHypothesis = hypotheses.Element[interpretation.Element[iHypothesis]];

        if (pHypothesis->pVN->featureArray.n > maxnVNFeatures)
            maxnVNFeatures = pHypothesis->pVN->featureArray.n;
    }

    float *SDF = new float[maxnVNFeatures];

    float R[9];

    RVLUNITMX3(R);

    Rect<float> fROI;
    Rect<int> ROI;
    int iActiveFeature;
    float rM[3];
    VN_::SurfaceRayIntersection surfaceRayIntersection;

    for (iHypothesis = 0; iHypothesis < interpretation.n; iHypothesis++)
    {
        pHypothesis = hypotheses.Element[interpretation.Element[iHypothesis]];

        pSurfels->GetDepthImageROI(pHypothesis->iVertexArray, camera, fROI);

        ROI.minx = (int)(fROI.minx);
        ROI.maxx = (int)(fROI.maxx);
        ROI.miny = (int)(fROI.miny);
        ROI.maxy = (int)(fROI.maxy);

        ExpandRect<int>(&ROI, ROIBorderSize);

        CropRect<int>(ROI, cropWin);

        for (v = ROI.miny; v <= ROI.maxy; v++)
            for (u = ROI.minx; u <= ROI.maxx; u++)
            {
                // if (u == 268 && v == 263)
                //	int debug = 0;

                iPix = u + v * camera.w;

                pPt = pMesh->NodeArray.Element + iPix;

                if (bValidPt[iPix])
                {
                    e = pHypothesis->pVN->Evaluate(pPt->P, SDF, iActiveFeature, true, pHypothesis->d);

                    e = RVLABS(e);

                    if (objectMap.Element[iPix] == -2 || e < ez[iPix])
                    {
                        ez[iPix] = e;

                        objectMap.Element[iPix] = iHypothesis;
                    }
                }
                else
                {
                    r[0] = ((float)u - camera.uc) / camera.fu;
                    r[1] = ((float)v - camera.vc) / camera.fv;
                    r[2] = 1.0f;

                    RVLNORM3(r, fTmp);

                    RVLMULMX3X3TVECT(R, r, rM);

                    RVLNORM3(rM, fTmp);

                    surfaceRayIntersection = pHypothesis->pVN->Project(pHypothesis->d, rM);

                    z = surfaceRayIntersection.s;

                    if (z < ez[iPix])
                    {
                        ez[iPix] = z;

                        objectMap.Element[iPix] = iHypothesis;
                    }
                }
            }
    }

    delete[] ez;
    delete[] SDF;
    delete[] bValidPt;

#ifdef NEVER
    Rect<int> *ROI = new Rect<int>[interpretation.n];

    Rect<int> cropWin;
    cropWin.minx = 0;
    cropWin.maxx = camera.w - 1;
    cropWin.miny = 0;
    cropWin.maxy = camera.h - 1;

    int nPixels = camera.w * camera.h;

    int nAssignments = nPixels;

    int iHypothesis;
    VN_::Hypothesis *pHypothesis;
    Rect<float> fROI;
    Rect<int> *pROI;

    for (iHypothesis = 0; iHypothesis < interpretation.n; iHypothesis++)
    {
        pHypothesis = interpretation.Element[iHypothesis];

        pSurfels->GetDepthImageROI(pHypothesis->iVertexArray, camera, fROI);

        pROI = ROI + iHypothesis;

        pROI->minx = (int)(fROI.minx);
        pROI->maxx = (int)(fROI.maxx);
        pROI->miny = (int)(fROI.miny);
        pROI->maxy = (int)(fROI.maxy);

        ExpandRect<int>(pROI, ROIBorderSize);

        CropRect<int>(*pROI, cropWin);

        nAssignments += Area(*pROI);
    }

    QLIST::Entry<Pair<int, float>> *assignmentMem = new QLIST::Entry<Pair<int, float>>[nAssignments];

    QLIST::Entry<Pair<int, float>> *pAssignment = assignmentMem;

    QList<QLIST::Entry<Pair<int, float>>> *assignmentList = new QList<QLIST::Entry<Pair<int, float>>>[nPixels];

    QList<QLIST::Entry<Pair<int, float>>> *pAssignmentList = assignmentList;

    int iPix = 0;

    int u, v;
    float s, z, fTmp;
    float r[3];
    Point *pPt;

    for (v = 0; v < camera.h; v++)
        for (u = 0; u < camera.w; u++, iPix++, pAssignmentList++)
        {
            RVLQLIST_INIT(pAssignmentList);

            r[0] = ((float)u - camera.uc) / camera.fu;
            r[1] = ((float)v - camera.vc) / camera.fv;
            r[2] = 1.0f;

            fTmp = RVLDOTPRODUCT3(NGnd, r);

            if (fTmp > 1e-10)
            {
                s = dGnd / fTmp;

                z = s * r[2];

                if (z <= maxz)
                {
                    RVLQLIST_ADD_ENTRY(pAssignmentList, pAssignment);

                    pAssignment->data.a = -1;

                    pPt = pMesh->NodeArray.Element + iPix;

                    pAssignment->data.b = (RVLDOTPRODUCT3(pPt->N, pPt->N) >= 1e-10 ? RVLDOTPRODUCT3(NGnd, pPt->P) - dGnd : z);

                    pAssignment++;
                }
            }
        }

    for (iHypothesis = 0; iHypothesis < interpretation.n; iHypothesis++)
    {
        pHypothesis = interpretation.Element[iHypothesis];

        pROI = ROI + iHypothesis;

        for (v = pROI->miny; v <= pROI->maxy; v++)
            for (u = pROI->minx; u <= pROI->maxx; u++)
            {
                iPix = u + v * camera.w;

                pPt = pMesh->NodeArray.Element + iPix;

                pAssignmentList = assignmentList + iPix;

                if (RVLDOTPRODUCT3(pPt->N, pPt->N) >= 1e-10)
                {
                    pHypothesis->pVN->Evaluate(pPt->P, SDF, pHypothesis->d);
                }
            }
    }

    delete[] assignmentList;
    delete[] assignmentMem;
#endif
}

void VNClassifier::SuperSegmentNeighbourhood(Mesh *pMesh)
{
    // Identify all surfels which belong to a supersegment.

    bool *bSurfelAssigned = new bool[pSurfels->NodeArray.n];

    memset(bSurfelAssigned, 0, pSurfels->NodeArray.n * sizeof(bool));

    int i, iSuperSegment;
    VN_::SuperSegment *pSuperSegment;

    for (iSuperSegment = 0; iSuperSegment < superSegments.n; iSuperSegment++)
    {
        pSuperSegment = superSegments.Element[iSuperSegment];

        for (i = 0; i < pSuperSegment->iSurfelArray.n; i++)
            bSurfelAssigned[pSuperSegment->iSurfelArray.Element[i]] = true;
    }

    // ptArray <- all mesh points assigned to any supersegment

    Array<Point *> ptArray;

    ptArray.Element = new Point *[pMesh->NodeArray.n];
    ptArray.n = 0;

    int iPt;

    for (int iPt = 0; iPt < pMesh->NodeArray.n; iPt++)
        if (pSurfels->surfelMap[iPt] >= 0)
            if (bSurfelAssigned[pSurfels->surfelMap[iPt]])
                ptArray.Element[ptArray.n++] = pMesh->NodeArray.Element + iPt;

    delete[] bSurfelAssigned;

    // Assign ptArray to pNeighborhoodTool.

    pNeighborhoodTool->Create(&ptArray);

    // Compute supersegment centroids.

    Surfel *pSurfel;
    RVL::QLIST::Index2 *pt;

    // Finding centroids
    float *centroids = new float[3 * superSegments.n];
    memset(centroids, 0, 3 * superSegments.n * sizeof(float));
    int noPts;
    for (iSuperSegment = 0; iSuperSegment < superSegments.n; iSuperSegment++)
    {
        pSuperSegment = superSegments.Element[iSuperSegment];
        noPts = 0;
        for (int i = 0; i < pSuperSegment->iSurfelArray.n; i++)
        {
            pSurfel = pSurfels->NodeArray.Element + pSuperSegment->iSurfelArray.Element[i];
            pt = pSurfel->PtList.pFirst;
            while (pt)
            {
                centroids[3 * iSuperSegment] += pMesh->NodeArray.Element[pt->Idx].P[0];
                centroids[3 * iSuperSegment + 1] += pMesh->NodeArray.Element[pt->Idx].P[1];
                centroids[3 * iSuperSegment + 2] += pMesh->NodeArray.Element[pt->Idx].P[2];
                noPts++;
                pt = pt->pNext;
            }
        }
        centroids[3 * iSuperSegment] /= noPts;
        centroids[3 * iSuperSegment + 1] /= noPts;
        centroids[3 * iSuperSegment + 2] /= noPts;
    }

    // Debug

    // FILE *fpRadiusSearchTest = fopen("C:\\RVL\\Debug\\P.txt", "w");

    // fprintf(fpRadiusSearchTest, "%f\t%f\t%f\n", centroids[0], centroids[1], centroids[2]);

    // float *Pn;

    // for (int i = 0; i < pointIdxRadiusSearch.size(); i++)
    //{
    //	Pn = pMesh->NodeArray.Element[pointIdxRadiusSearch[i]].P;

    //	fprintf(fpRadiusSearchTest, "%f\t%f\t%f\n", Pn[0], Pn[1], Pn[2]);
    //}

    // for (int i = 0; i < ptArray.n; i++)
    //{
    //	Pn = ptArray.Element[i]->P;

    //	fprintf(fpRadiusSearchTest, "%f\t%f\t%f\n", Pn[0], Pn[1], Pn[2]);
    //}

    // fclose(fpRadiusSearchTest);

    //

    segmentN_PD.clear();

    std::vector<int> pointIdxRadiusSearch;         // to store index of surrounding points
    std::vector<float> pointRadiusSquaredDistance; // to store distance to surrounding points
    vtkSmartPointer<vtkPoints> points;
    vtkSmartPointer<vtkFloatArray> normals;
    vtkSmartPointer<vtkCellArray> verts;
    vtkSmartPointer<vtkPolyData> PD;
    int ptIdx = 0;

    Point *pPt;

    for (iSuperSegment = 0; iSuperSegment < superSegments.n; iSuperSegment++)
    {
        points = vtkSmartPointer<vtkPoints>::New();
        normals = vtkSmartPointer<vtkFloatArray>::New();
        normals->SetNumberOfComponents(3);
        verts = vtkSmartPointer<vtkCellArray>::New();

        pointIdxRadiusSearch.clear();
        pointRadiusSquaredDistance.clear();

        pNeighborhoodTool->RadiusSearch(centroids + 3 * iSuperSegment, ICPSuperSegmentRadius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

        ptIdx = 0;
        for (int i = 0; i < pointIdxRadiusSearch.size(); i++)
        {
            pPt = ptArray.Element[pointIdxRadiusSearch[i]];

            points->InsertNextPoint(pPt->P);
            normals->InsertNextTuple(pPt->N);
            verts->InsertNextCell(1);
            verts->InsertCellPoint(ptIdx);
            ptIdx++;
        }

        PD = vtkSmartPointer<vtkPolyData>::New();
        PD->SetPoints(points);
        PD->GetPointData()->SetNormals(normals);
        PD->SetVerts(verts);

        // subsampling the scene:
        vtkSmartPointer<vtkCleanPolyData> cleanFilter = vtkSmartPointer<vtkCleanPolyData>::New();
        cleanFilter->SetInputData(PD);
        cleanFilter->PointMergingOn();
        // cleanFilter->SetAbsoluteTolerance(0.005);
        cleanFilter->SetAbsoluteTolerance(0.005);
        cleanFilter->ToleranceIsAbsoluteOn();
        cleanFilter->Update();

        segmentN_PD.insert(std::make_pair(iSuperSegment, cleanFilter->GetOutput()));
    }
    delete[] centroids;
}

void VNClassifier::ComputeClassPropertiesForGrasping(int iClass, VN *pModel)
{
}
void VNClassifier::SampleSO3()
{
    int nRotSamples = 16 * sampledUnitSphere.h;

    SO3Samples.a = 3;
    SO3Samples.b = 3;
    SO3Samples.c = nRotSamples;

    SO3Samples.Element = new float[SO3Samples.a * SO3Samples.b * SO3Samples.c];

    float R[9];

    float *X = R;
    float *Y = R + 3;
    float *Z = R + 6;

    float dphi = 2.0f * PI / 16.0f;

    int iRotSample = 0;

    int i, j, k, i_;
    float *Z_;
    float X_[3], Y_[3], VTmp[3];
    float fTmp;
    float phi, cs, sn;
    float *R__;

    for (i = 0; i < sampledUnitSphere.h; i++)
    {
        Z_ = sampledUnitSphere.Element + sampledUnitSphere.w * i;

        RVLCOPY3VECTOR(Z_, Z);

        RVLORTHOGONAL3(Z_, X_, i_, j, k, fTmp);

        RVLCROSSPRODUCT3(Z_, X_, Y_);

        for (j = 0; j < 16; j++)
        {
            phi = (float)j * dphi;

            cs = cos(phi);
            sn = sin(phi);

            RVLSCALE3VECTOR(X_, cs, X);
            RVLSCALE3VECTOR(Y_, -sn, VTmp);
            RVLSUM3VECTORS(X, VTmp, X);

            RVLSCALE3VECTOR(X_, sn, Y);
            RVLSCALE3VECTOR(Y_, cs, VTmp);
            RVLSUM3VECTORS(Y, VTmp, Y);

            R__ = SO3Samples.Element + 9 * iRotSample;

            RVLCOPYMX3X3(R, R__);

            iRotSample++;
        }
    }
}

float VNClassifier::ProjectToLatentSubspace(
    float *d,
    uchar *bd,
    float *dU,
    uchar *bdU,
    ClassData *pClass,
    float *qOpt,
    int &iR)
{
    VN *pModel = models[pClass->iMetaModel];

    Array2D<float> M = pClass->M;

    int m = M.h;

    int m2 = m * m;

    Array<int> iV;

    iV.Element = new int[M.w];

    cv::Mat A;

    A.create(m, m, CV_32FC1);

    float *A_ = (float *)(A.data);

    cv::Mat b;

    b.create(m, 1, CV_32FC1);

    float *b_ = (float *)(b.data);

    float *a = new float[m];

    cv::Mat q;

    q.create(m, 1, CV_32FC1);

    float *q_ = (float *)(q.data);

    float *dSV = new float[M.w];

    float minE = -1.0f;

    int i, j, l, iN;
    float *d__, *dU__, *dS;
    uchar *bd__, *bdU__, *bdS;
    int iFeature;
    float *M_;
    float dS_;
    VN_::ModelCluster *pModelCluster;
    float dM, e, E;

    // for (int k = 0; k < 1000; k++)
    //{
    for (i = 0; i < SO3Samples.c; i++)
    {
        d__ = d + i * sampledUnitSphere.h;
        bd__ = bd + i * sampledUnitSphere.h;
        dU__ = dU + i * sampledUnitSphere.h;
        bdU__ = bdU + i * sampledUnitSphere.h;

        memset(A_, 0, m2 * sizeof(float));

        memset(b_, 0, m * sizeof(float));

        iV.n = 0;

        pModelCluster = pModel->modelClusterList.pFirst;

        while (pModelCluster)
        {
            if (pModelCluster->type == RVLVN_CLUSTER_TYPE_CONVEX)
            {
                dS = d__;
                bdS = bd__;
            }
            else if (pModelCluster->type == RVLVN_CLUSTER_TYPE_CONCAVE)
            {
                dS = dU__;
                bdS = bdU__;
            }

            for (iFeature = pModelCluster->iFeatureInterval.a; iFeature <= pModelCluster->iFeatureInterval.b; iFeature++)
            {
                iN = pModelCluster->iN[iFeature - pModelCluster->iFeatureInterval.a];

                if (bdS[iN])
                {
                    M_ = M.Element + iFeature;

                    for (j = 0; j < m; j++)
                        a[j] = M_[j * M.w];

                    for (l = 0; l < m; l++)
                        for (j = l; j < m; j++)
                            A_[l * m + j] += (a[l] * a[j]);

                    dS_ = dS[iN];

                    for (j = 0; j < m; j++)
                        b_[j] += (a[j] * dS_);

                    iV.Element[iV.n] = iFeature;

                    dSV[iV.n] = dS_;

                    iV.n++;
                }
            }

            pModelCluster = pModelCluster->pNext;
        }

        for (l = 1; l < m; l++)
            for (j = 0; j < l; j++)
                A_[l * m + j] = A_[j * m + l];

        cv::solve(A, b, q);

        E = 0.0f;

        for (j = 0; j < iV.n; j++)
        {
            M_ = M.Element + iV.Element[j];

            dM = 0.0f;

            for (l = 0; l < m; l++)
                dM += M_[l * M.w] * q_[l];

            e = dSV[j] - dM;

            E += e * e;
        }

        if (minE < 0.0f || E < minE)
        {
            minE = E;

            iR = i;

            memcpy(qOpt, q_, M.h * sizeof(float));
        }
    } // for every descriptor

    delete[] a;
    delete[] iV.Element;
    delete[] dSV;

    return minE;
}

float VNClassifier::ProjectToLatentSubspace(
    float *d,
    uchar *bd,
    RECOG::ClassData *pClass,
    RECOG::VN_::FitData *pFitData,
    float *qOpt,
    int &iR,
    int &nb,
    bool &bAllDoFsVisible)
{
    VN *pModel = models[pClass->iMetaModel];

    // int nqs = M.h - 4;

    // float *qs = new float[nqs];

    // float *qs_ = q_ + 4;

    float minE = -1.0f;

    bAllDoFsVisible = false;

    // printf("lambda=%f, sqrtbeta=%f\n", lambda_, sqrtbeta);

    // FILE *fpDebug = fopen("C:\\RVL\\ExpRez\\q.txt", "w");

    // FILE *fpDebug2 = fopen("C:\\RVL\\ExpRez\\A.txt", "w");

    int i;
    float *dS;
    uchar *bdS;
    float E;

    // for (int k = 0; k < 1000; k++)
    //{
    for (i = 0; i < SO3Samples.c; i++)
    {
        // if (i == 272)
        //	int debug = 0;

        dS = d + i * sampledUnitSphere.h;
        bdS = bd + i * sampledUnitSphere.h;

        if (FitLS(dS, bdS, pClass, pFitData, E))
        {
            bAllDoFsVisible = true;

            if (minE < 0.0f || E < minE)
            {
                minE = E;

                iR = i;

                memcpy(qOpt, pFitData->q_, pClass->M.h * sizeof(float));

                nb = pFitData->iV.n;
            }
        }
    } // for every descriptor

    // fclose(fpDebug);
    // fclose(fpDebug2);

    // delete[] qs;

    return minE;
}

// Requires prealocated variable allbuffer since results wil be saved into her sequentially. (tuple<E, q, iR, nb, bAllDoFsVisible>)
void VNClassifier::ProjectToLatentSubspace(
    float *d,
    uchar *bd,
    RECOG::ClassData *pClass,
    RECOG::VN_::FitData *pFitData,
    std::vector<std::tuple<float, float *, int, int, bool>> *allbuffer)
{
    VN *pModel = models[pClass->iMetaModel];

    float minE = -1.0f;

    bool bAllDoFsVisible = false;

    int i;
    float *dS;
    uchar *bdS;
    float E;

    for (i = 0; i < SO3Samples.c; i++)
    {
        dS = d + i * sampledUnitSphere.h;
        bdS = bd + i * sampledUnitSphere.h;

        if (FitLS(dS, bdS, pClass, pFitData, E))
        {
            bAllDoFsVisible = true;

            std::get<0>((*allbuffer)[i]) = E;
            memcpy(std::get<1>((*allbuffer)[i]), pFitData->q_, pClass->M.h * sizeof(float));
            std::get<2>((*allbuffer)[i]) = i;
            std::get<3>((*allbuffer)[i]) = pFitData->iV.n;
            std::get<4>((*allbuffer)[i]) = bAllDoFsVisible;
        }
    } // for every descriptor
}

float VNClassifier::FitLS(
    float *d1,
    uchar *bd1,
    float *d2,
    uchar *bd2,
    float *t_,
    int &nV,
    float *e)
{
    cv::Mat A(3, 3, CV_32FC1);
    float *A_ = (float *)(A.data);
    RVLNULLMX3X3(A_);

    cv::Mat b(3, 1, CV_32FC1);
    float *b_ = (float *)(b.data);
    RVLNULL3VECTOR(b_);

    cv::Mat t(3, 1, CV_32FC1, t_);

    nV = 0;

    float *a;
    int j, l;
    int iFeature;
    float e_;
    float b__[3];
    float A__[9];

    for (iFeature = 0; iFeature < sampledUnitSphere.h; iFeature++)
    {
        if (!(bd1[iFeature] && bd2[iFeature]))
            continue;

        nV++;

        e_ = d2[iFeature] - d1[iFeature];

        a = sampledUnitSphere.Element + 3 * iFeature;

        RVLVECTCOV3(a, A__);

        RVLSUMMX3X3UT(A_, A__, A_);

        RVLSCALE3VECTOR(a, e_, b__);

        RVLSUM3VECTORS(b_, b__, b_);
    }

    if (nV < 3)
        return -1.0f;

    RVLCOMPLETESIMMX3(A_);

    cv::solve(A, b, t);

    float E = 0.0f;

    for (iFeature = 0; iFeature < sampledUnitSphere.h; iFeature++)
    {
        a = sampledUnitSphere.Element + 3 * iFeature;

        e_ = d2[iFeature] - d1[iFeature] - RVLDOTPRODUCT3(a, t_);

        if (e)
            e[iFeature] = e_;

        if (bd1[iFeature] && bd2[iFeature])
            E += e_ * e_;
    }

    return E;
}

float VNClassifier::FitLS(
    float *A,
    float *d1,
    float *d2,
    uchar *bd,
    float *t,
    float *e)
{
    float b[3];
    RVLNULL3VECTOR(b);

    float *a;
    int j, l;
    int iFeature;
    float e_;
    float b__[3];
    float A__[9];

    for (iFeature = 0; iFeature < sampledUnitSphere.h; iFeature++)
    {
        if (!bd[iFeature])
            continue;

        e_ = d2[iFeature] - d1[iFeature];

        a = sampledUnitSphere.Element + 3 * iFeature;

        RVLSCALE3VECTOR(a, e_, b__);

        RVLSUM3VECTORS(b, b__, b);
    }

    RVLMULMX3X3VECT(A, b, t);

    float E = 0.0f;

    for (iFeature = 0; iFeature < sampledUnitSphere.h; iFeature++)
    {
        a = sampledUnitSphere.Element + 3 * iFeature;

        e_ = d2[iFeature] - d1[iFeature] - RVLDOTPRODUCT3(a, t);

        if (e)
            e[iFeature] = e_;

        if (bd[iFeature])
            E += e_ * e_;
    }

    return E;
}

int VNClassifier::InitFitLS(
    float *d,
    uchar *bd,
    float *A_)
{
    float A[9];
    RVLNULLMX3X3(A);

    int nV = 0;

    float *a;
    int j, l;
    int iFeature;
    float A__[9];

    for (iFeature = 0; iFeature < sampledUnitSphere.h; iFeature++)
    {
        if (!bd[iFeature])
            continue;

        nV++;

        a = sampledUnitSphere.Element + 3 * iFeature;

        RVLVECTCOV3(a, A__);

        RVLSUMMX3X3UT(A, A__, A);
    }

    if (nV >= 3)
    {
        float detA;

        RVLINVCOV3(A, A_, detA);

        RVLCOMPLETESIMMX3(A_);
    }

    return nV;
}

bool VNClassifier::FitLS(
    float *d,
    uchar *bd,
    RECOG::ClassData *pClass,
    VN_::FitData *pData,
    float &E)
{
    Array2D<float> M = pClass->M;
    float *A_ = pData->A_;
    float *b_ = pData->b_;
    float *q_ = pData->q_;
    float *a = pData->a;
    int m = M.h;

    memset(A_, 0, m * m * sizeof(float));

    memset(b_, 0, m * sizeof(float));

    pData->iV.n = 0;
    float w = 0.0f;
    float fn = (float)(M.w);
    bool bFullRank = false;

    int i, j, l;
    int iFeature;
    float dS_;
    float beta__, lambda__;
    float *M_;
    // float *AH;
    float dM, e, maxe;

    for (iFeature = 0; iFeature < M.w; iFeature++)
    {
        dS_ = d[iFeature];

        // if (bdS[iFeature])
        if (!bDepthImage || dS_ < 0.0f)
        {
            if (!bDepthImage || bd[iFeature])
            {
                beta__ = 1.0f;

                pData->iV.Element[pData->iV.n] = iFeature;

                pData->dSV[pData->iV.n] = dS_;

                pData->iV.n++;
            }
            else
                beta__ = pData->sqrtbeta;

            M_ = M.Element + iFeature;

            for (j = 0; j < m; j++)
                a[j] = beta__ * M_[j * M.w];

            for (l = 0; l < m; l++)
                for (j = l; j < m; j++)
                    A_[l * m + j] += (a[l] * a[j]);

            for (j = 0; j < m; j++)
                b_[j] += (beta__ * a[j] * dS_);

            w += (beta__ * beta__);
        }
    }

    lambda__ = pData->lambda1 * (fn - w) + pData->lambda0;

    for (l = 1; l < m; l++)
    {
        for (j = 0; j < l; j++)
            A_[l * m + j] = A_[j * m + l];

        if (l > fitParams.regularization)
            A_[l * m + l] += lambda__;
    }

    // PrintMatrix<float>(fpDebug2, A_, 1, m2);

    if (cv::solve(pData->A, pData->b, pData->q))
    {
        // PrintMatrix<float>(fpDebug, q_, 1, m);

        if (q_[3] >= 0.0f)
        {
            bFullRank = true;

            E = 0.0f;

            for (j = 0; j < pData->iV.n; j++)
            {
                M_ = M.Element + pData->iV.Element[j];

                dM = 0.0f;

                for (l = 0; l < m; l++)
                    dM += (M_[l * M.w] * q_[l]);

                e = pData->dSV[j] - dM;

                E += (e * e);
            }

            // E /= ((float)(pData->iV.n));

            // s = q_[3];

            ////RVLSCALEVECTOR(qs_, s, qs, nqs, j);

            // maxe = 0.0f;

            // for (j = 0; j < pClass->nHull; j++)
            //{
            //	AH = pClass->A + nqs * j;

            //	RVLDOTPRODUCT(AH, qs, nqs, e, l);

            //	e -= (s * pClass->b[j]);

            //	if (e > maxe)
            //		maxe = e;
            //}

            // E += (maxe * maxe);

            return true;
        } // If q[3] > 0
        else
            return false;
    } // If system A, b, q is solvable.
    else
        return false;
}

bool VNClassifier::FitRotLS(
    Array<int> iVertexArray,
    float o,
    float *NM,
    RECOG::ClassData *pClass,
    float *RIn,
    int nIterations,
    bool bNormalTest,
    bool bVisibilityTest,
    RECOG::VN_::FitData *pData,
    float *qOpt,
    float *ROpt,
    float &E)
{
    float lambdaRotUp = 2.0f;
    float lambdaRotDown = 1.0f / lambdaRotUp;

    Array2D<float> M = pClass->M;
    float *A_ = pData->A_;
    float *b_ = pData->b_;
    float *x = pData->q_;
    float *q_ = x + 3;
    float *a = pData->a;
    float *aq = a + 3;
    int m = M.h;
    int nx = m + 3;

    // R <- RIn

    float R[9];

    RVLCOPYMX3X3(RIn, R);

    // q <- q - [pData->T * R' * pData->Pc; zeros(m - 3, 1)]

    float PcM[3];

    RVLMULMX3X3TVECT(R, pData->Pc, PcM);

    float qc[3];

    float *T = pClass->T;

    RVLMULMX3X3VECT(T, PcM, qc);

    RVLDIF3VECTORS(q_, qc, q_);

    // main loop

    memcpy(qOpt, q_, m * sizeof(float));

    // pData->iV.n = 0;
    float fn = (float)(M.w);
    bool bFullRank = false;
    float E_ = -1.0f;

    int i, j, k, l;
    int iFeature;
    float dS, dSc;
    float beta__, lambdaRot, lambda__;
    float *M_;
    // float *AH;
    float dM, e, maxe, dSmax, d_, dSmaxValid;
    float NS[3];
    float *NM_;
    SURFEL::Vertex *pVertex;
    float *P;
    bool bdmax, bdmaxValid;
    float distFromNHull;
    bool bd;
    int iVertex, iVertex_;
    float th;
    float z[3];
    float dR[9], R_[9], R__[9], V3Tmp[3];
    float w;
    float maxTrA, fTmp;
    int nb;
    float beta2;

    for (k = 0; k < nIterations; k++)
    {
        w = 0.0f;
        E = 0.0f;
        nb = 0;

        memset(A_, 0, nx * nx * sizeof(float));

        memset(b_, 0, nx * sizeof(float));

        // FILE *fpDebug = fopen("C:\\RVL\\ExpRez\\a_.txt", "w");
        // FILE *fpDebug3 = fopen("C:\\RVL\\ExpRez\\P.txt", "w");
        // FILE *fpDebug4 = fopen("C:\\RVL\\ExpRez\\debug.txt", "w");

        for (iFeature = 0; iFeature < M.w; iFeature++)
        {
            // fprintf(fpDebug4, "iFeature=%d\n", iFeature);

            NM_ = NM + 3 * iFeature;

            RVLMULMX3X3VECT(R, NM_, NS);

            bdmax = bdmaxValid = false;

            for (j = 0; j < iVertexArray.n; j++)
            {
                iVertex_ = iVertexArray.Element[j];

                // fprintf(fpDebug4, "iVertex_=%d\n", iVertex_);

                pVertex = pSurfels->vertexArray.Element[iVertex_];

                P = pVertex->P;

                d_ = RVLDOTPRODUCT3(NS, P);

                if (bdmax)
                {
                    if (o * d_ > o * dSmax)
                    {
                        dSmax = d_;

                        iVertex = iVertex_;
                    }
                }
                else
                {
                    dSmax = d_;

                    iVertex = iVertex_;

                    bdmax = true;
                }

                if (bNormalTest)
                {
                    if (d_ < 0.0f || !bVisibilityTest)
                    {
                        distFromNHull = pSurfels->DistanceFromNormalHull(pVertex->normalHull, NS);

                        if (distFromNHull <= maxDistFromNHull)
                        {
                            if (bdmaxValid)
                            {
                                if (o * d_ > o * dSmaxValid)
                                    dSmaxValid = d_;
                            }
                            else
                            {
                                dSmaxValid = d_;

                                bdmaxValid = true;
                            }
                        }
                    }
                }
            } // for each vertex

            if (dSmax >= 0.0f)
                continue;

            bd = false;

            if (bdmaxValid)
                bd = (o * (dSmax - dSmaxValid) <= maxedmax);

            // Comment this!
            // bd = true;

            if (bd)
            {
                beta__ = 1.0f;

                // pData->iV.Element[pData->iV.n] = iFeature;

                // pData->dSV[pData->iV.n] = dS;

                // pData->iV.n++;
            }
            else
                beta__ = pData->sqrtbeta;

            beta2 = beta__ * beta__;

            dS = dSmax;

            dSc = dS - RVLDOTPRODUCT3(NS, pData->Pc);

            P = pData->P + 3 * iVertex;

            // fprintf(fpDebug3, "%d\t", iVertex);

            // PrintMatrix<float>(fpDebug3, P, 1, 3);

            RVLCROSSPRODUCT3(P, NS, a);

            RVLSCALE3VECTOR(a, beta__, a);

            M_ = M.Element + iFeature;

            for (j = 0; j < m; j++)
                aq[j] = beta__ * M_[j * M.w];

            for (l = 0; l < nx; l++)
                for (j = l; j < nx; j++)
                    A_[l * nx + j] += (a[l] * a[j]);

            // PrintMatrix<float>(fpDebug, a, 1, nx);

            // fTmp = beta__ * e;
            fTmp = beta__ * dSc;

            RVLSCALE3VECTOR(a, fTmp, V3Tmp);

            RVLSUM3VECTORS(b_, V3Tmp, b_);

            for (j = 3; j < nx; j++)
                b_[j] += (beta__ * a[j] * dSc);

            w += beta2;

            dM = 0.0f;

            for (l = 0; l < m; l++)
                dM += (M_[l * M.w] * q_[l]);

            e = dSc - dM;

            // if (bd)
            {
                E += (beta2 * e * e);
                nb++;
            }
        } // for (iFeature = 0; iFeature < M.w; iFeature++)

        // fclose(fpDebug);
        // fclose(fpDebug3);
        // fclose(fpDebug4);

        E /= ((float)nb);

        if (E_ > 0.0f && E > E_)
        {
            RVLCOPYMX3X3(R_, R);

            if (k >= nIterations - 1)
                break;
            else
                lambdaRot *= lambdaRotUp;
        }
        else if (E_ < 0.0f)
        {
            maxTrA = 0.0;

            for (i = 3; i < nx; i++)
            {
                fTmp = A_[i * nx + i];

                if (fTmp > maxTrA)
                    maxTrA = fTmp;
            }

            lambdaRot = 0.1f * maxTrA;

            E_ = E;

            RVLCOPYMX3X3(R, R_);
        }
        else
        {
            lambdaRot *= lambdaRotDown;

            E_ = E;

            RVLCOPYMX3X3(R, R_);

            memcpy(qOpt, q_, m * sizeof(float));
        }

        if (k >= nIterations)
            break;

        lambda__ = pData->lambda1 * (fn - w) + pData->lambda0;

        for (l = 0; l < 3; l++)
            A_[l * nx + l] += lambdaRot;

        for (l = 1; l < nx; l++)
        {
            for (j = 0; j < l; j++)
                A_[l * nx + j] = A_[j * nx + l];

            if (l > 6)
                A_[l * nx + l] += lambda__;
        }

        // FILE *fpDebug2 = fopen("C:\\RVL\\ExpRez\\A.txt", "w");

        // PrintMatrix<float>(fpDebug2, A_, nx, nx);

        // fclose(fpDebug2);

        if (!cv::solve(pData->A, pData->b, pData->q))
        {
            RVLCOPYMX3X3(R_, R);

            break;
        }

        // int k_ = 0;

        // while (!cv::solve(pData->A, pData->b, pData->q) && k_ < 10)
        //{
        //	for (l = 0; l < 3; l++)
        //		A_[l * nx + l] += lambdaRot;

        //	lambdaRot *= lambdaRotUp;

        //	k_++;
        //}

        // if (k_ >= 10)
        //	return false;

        // Only for debugging purpose!

        ///

        if (q_[3] < 0.0f)
            return false;

        // PrintMatrix<float>(fpDebug, q_, 1, m);

        bFullRank = true;

        // dphi = || dphi ||

        th = sqrt(RVLDOTPRODUCT3(x, x));

        // printf("th=%f\n", th);

        if (RVLABS(th) >= 1e-8)
        {
            // z <- dphi / || dphi ||

            RVLSCALE3VECTOR2(x, th, z);

            // dR <- Rot(z, th) (angle axis to rotation matrix)

            AngleAxisToRot<float>(z, th, dR);

            // R <- dR * R

            RVLMXMUL3X3(dR, R, R__);

            RVLCOPYMX3X3(R__, R);
        }
    } // for (k = 0; k < nIterations; k++)

    // ROpt <- R

    RVLCOPYMX3X3(R, ROpt);

    // q <- q - [pData->T * R' * pData->Pc; zeros(m - 3, 1)]

    RVLMULMX3X3TVECT(ROpt, pData->Pc, PcM);

    RVLMULMX3X3VECT(T, PcM, qc);

    RVLSUM3VECTORS(qOpt, qc, qOpt);

    return true;
}

// Mathematics for this function is explained in ARP3D.TR4.11, CTI Fitting II

float VNClassifier::FitCTI(
    Mesh *pMesh,
    Array<int> iSurfelArray,
    Array<int> iVertexArray,
    Camera camera,
    int nIterations,
    float *R,
    float *d,
    uchar *bd)
{
    nIterations = 5;

    float lambdaUp = 2.0f;
    float lambdaDown = 1.0f / lambdaUp;
    float lambda = 0.1f;

    SURFEL::SceneSamples sceneSamples;

    sceneSamples.PGnd = NULL;

    pSurfels->SampleSurfelSet(pMesh, iSurfelArray, iVertexArray, camera, sceneSamples, true, false);

    float R0[9];

    RVLUNITMX3(R0);

    float *d_ = new float[sampledUnitSphere.h];

    uchar *bd_ = new uchar[sampledUnitSphere.h];

    float *d__ = new float[sampledUnitSphere.h];

    float *NS = new float[sampledUnitSphere.h * sampledUnitSphere.w];

    int *iV = new int[sampledUnitSphere.h];

    cv::Mat A;

    A.create(3, 3, CV_32FC1);

    float *A_ = (float *)(A.data);

    cv::Mat b;

    b.create(3, 1, CV_32FC1);

    float *b_ = (float *)(b.data);

    cv::Mat x(3, 1, CV_32FC1);

    float *x_ = (float *)(x.data);

    Array<SURFEL::NormalHullElement> NHull;

    NHull.Element = NULL;
    NHull.n = 0;

    // float *dFace = new float[sampledUnitSphere.h];

    int *nFace = new int[sampledUnitSphere.h];

    float *PcFace = new float[3 * sampledUnitSphere.h];

    int *iAssociatedFace = new int[sceneSamples.PtArray.n];

    // float U[5][3] = {
    //	{ 0.0f, 0.0f, 0.0f },
    //	{ 1.0f, 0.0f, 0.0f },
    //	{ 0.0f, 1.0f, 0.0f },
    //	{ -1.0f, 0.0f, 0.0f },
    //	{ 0.0f, -1.0f, 0.0f } };

    // float phi = 0.125 * PI;

    int iZ_[3] = {6, 8, 57};

    float *Z0 = R0 + 6;

    float EOpt = -1.0f;

    int i, k, iX, iZ;
    int iFace;
    float *P;
    float *NM, *NS_, *PcFace_, *Z;
    float e, e2, maxe, th, E, E_, fnFace, phi, sf, ca, sa, al;
    bool bFirst;
    int iAssociatedFace_;
    float dP[3], J[3], A__[9], b__[3], z[3], dR[9], R_[9], R__[9], ROpt[9], RZ[9], RX[9], V3Tmp[3];
    SURFEL::Vertex *pVertex;

    for (iZ = 0; iZ < 4; iZ++)
    // iZ = 3;
    {
        if (iZ == 0)
        {
            RVLUNITMX3(RZ)
        }
        else
        {
            Z = sampledUnitSphere.Element + 3 * iZ_[iZ - 1];

            RVLCROSSPRODUCT3(Z0, Z, V3Tmp);

            RVLNORM3(V3Tmp, sf);

            phi = asin(sf);

            AngleAxisToRot<float>(V3Tmp, phi, RZ);
        }

        for (iX = 0; iX < 3; iX++)
        // iX = 1;
        {
            al = 0.125 * PI * (float)iX;

            ca = cos(al);
            sa = sin(al);

            RVLROTZ(ca, sa, RX);

            RVLMXMUL3X3(RZ, RX, R);

            // float RGT_[9];

            // float *XGT_ = RGT_;
            // float *YGT_ = RGT_ + 3;
            // float *ZGT_ = RGT_ + 6;

            // RVLSET3VECTOR(ZGT_, -0.067668, -0.735154, -0.674515);
            // RVLSET3VECTOR(XGT_, -0.51496, 0.560113, -0.615370);
            // RVLCROSSPRODUCT3(ZGT_, XGT_, YGT_);

            // float debug = RVLMULROWCOL3(RGT_, R, 0, 0);

            // float RGT[9];

            // RVLCOPYMX3X3T(RGT_, RGT);

            // GetAngleAxis(RGT, z, phi);

            // float fTmp;

            // RVLNORM3(z, fTmp);

            // AngleAxisToRot<float>(z, phi, dR);

            // RVLMXMUL3X3(dR, R0, R);

            // RVLCOPYMX3X3(R0, R);

            E_ = -1.0f;

            for (k = 0; k < nIterations; k++)
            {
                // if (k == 0)
                ComputeDescriptor(iVertexArray, NHull, R, d_, bd_, NS, iV);

                RVLNULLMX3X3(A_);

                RVLNULL3VECTOR(b_);

                // memset(dFace, 0, sampledUnitSphere.h * sizeof(float));

                memset(nFace, 0, sampledUnitSphere.h * sizeof(int));

                memset(PcFace, 0, 3 * sampledUnitSphere.h * sizeof(float));

                E = 0.0f;

                for (i = 0; i < sceneSamples.PtArray.n; i++)
                {
                    if (sceneSamples.status[i] != 1)
                        continue;

                    P = sceneSamples.PtArray.Element[i];

                    bFirst = true;

                    for (iFace = 0; iFace < sampledUnitSphere.h; iFace++)
                    {
                        if (d_[iFace] >= 0.0f)
                            continue;

                        NS_ = NS + 3 * iFace;

                        // NM = sampledUnitSphere.Element + 3 * iFace;

                        // RVLMULMX3X3VECT(R, NM, NS_);

                        e = RVLDOTPRODUCT3(NS_, P) - d_[iFace];

                        if (bFirst)
                        {
                            maxe = e;

                            iAssociatedFace_ = iFace;

                            bFirst = false;
                        }
                        else if (e > maxe)
                        {
                            maxe = e;

                            iAssociatedFace_ = iFace;
                        }
                    }

                    NS_ = NS + 3 * iAssociatedFace_;

                    // dFace[iAssociatedFace] += (RVLDOTPRODUCT3(NS_, P) - sceneSamples.g[i]);

                    nFace[iAssociatedFace_]++;

                    PcFace_ = PcFace + 3 * iAssociatedFace_;

                    RVLSUM3VECTORS(PcFace_, P, PcFace_);

                    iAssociatedFace[i] = iAssociatedFace_;
                } // for every sample point

                for (iFace = 0; iFace < sampledUnitSphere.h; iFace++)
                {
                    if (nFace[iFace] == 0)
                        continue;

                    fnFace = (float)(nFace[iFace]);

                    // d_[iFace] = dFace[iFace] / fnFace;

                    PcFace_ = PcFace + 3 * iFace;

                    RVLSCALE3VECTOR2(PcFace_, fnFace, PcFace_);

                    NS_ = NS + 3 * iFace;

                    d_[iFace] = RVLDOTPRODUCT3(NS_, PcFace_);
                }

                for (i = 0; i < sceneSamples.PtArray.n; i++)
                {
                    if (sceneSamples.status[i] != 1)
                        continue;

                    P = sceneSamples.PtArray.Element[i];

                    iAssociatedFace_ = iAssociatedFace[i];

                    // pVertex = pSurfels->vertexArray.Element[iV[iAssociatedFace]];

                    // RVLDIF3VECTORS(P, pVertex->P, dP);

                    PcFace_ = PcFace + 3 * iAssociatedFace_;

                    RVLDIF3VECTORS(P, PcFace_, dP);

                    NS_ = NS + 3 * iAssociatedFace_;

                    RVLCROSSPRODUCT3(dP, NS_, J);

                    e = RVLDOTPRODUCT3(NS_, dP) - sceneSamples.g[i];

                    E += (e * e);

                    RVLSCALE3VECTOR(J, e, b__);

                    RVLSUM3VECTORS(b_, b__, b_);

                    RVLVECTCOV3(J, A__);

                    RVLSUMMX3X3(A_, A__, A_);
                } // for every sample point

                if (E_ > 0.0f && E > E_)
                {
                    RVLCOPYMX3X3(R_, R);

                    if (k >= nIterations - 1)
                    {
                        memcpy(d_, d__, sampledUnitSphere.h * sizeof(float));

                        break;
                    }
                    else
                        lambda *= lambdaDown;
                }
                else
                {
                    // lambda *= lambdaUp;

                    if (lambda > 1.0f)
                        lambda = 1.0f;

                    E_ = E;

                    RVLCOPYMX3X3(R, R_);

                    memcpy(d__, d_, sampledUnitSphere.h * sizeof(float));
                }

                if (k >= nIterations - 1)
                    break;

                RVLCOMPLETESIMMX3(A_);

                cv::solve(A, b, x);

                // RVLSCALE3VECTOR(x_, lambda, x_);

                // dphi = || dphi ||

                th = sqrt(RVLDOTPRODUCT3(x_, x_));

                if (RVLABS(th) >= 1e-8)
                {
                    // z <- dphi / || dphi ||

                    RVLSCALE3VECTOR2(x_, th, z);

                    // dR <- Rot(z, th) (angle axis to rotation matrix)

                    if (th > 0.1f)
                        th = 0.1f;
                    else if (th < -0.1f)
                        th = -0.1f;

                    AngleAxisToRot<float>(z, th, dR);

                    // R <- dR * R

                    RVLMXMUL3X3(dR, R, R__);

                    RVLCOPYMX3X3(R__, R);
                }
                else
                    break;
            } // optimization iteration.

            if (EOpt < 0.0f || E < EOpt)
            {
                EOpt = E;

                RVLCOPYMX3X3(R, ROpt);

                memcpy(d, d_, sampledUnitSphere.h * sizeof(float));

                memcpy(bd, bd_, sampledUnitSphere.h * sizeof(uchar));
            }
        } // for (iX = 0; iX < 3; iX++)
    }     // for (iZ = 0; iZ < 4; iZ++)

    SURFEL::DeleteSceneSamples(sceneSamples);

    delete[] d_;
    delete[] bd_;
    delete[] d__;
    delete[] NS;
    delete[] iV;
    // delete[] dFace;
    delete[] nFace;
    delete[] PcFace;
    delete[] iAssociatedFace;

    RVLCOPYMX3X3(ROpt, R);

    return EOpt;
}

// Function FitCTI computes localy optimal rotation matrix R, which aligns convex template stored in sampledUnitSphere with
// the face normals of the convex mesh pConvexHull. The initial rotation matrix RInit is provided as an argument.
// The mathematics for FitCTI is described in ARP3D.TR3.9.

float VNClassifier::FitCTI(
    Mesh *pConvexHull,
    float *U,
    float *RInit,
    bool *mask,
    int nIterations,
    float *R,
    int *faceCorrespondence,
    Array<int> *piModelNormalArray)
{
    RVLCOPYMX3X3(RInit, R);

    float kR = 0.5f / (sigmaR * sigmaR);

    float V3Tmp2[3];
    float w, maxw;
    float xx, yy, zz, xy, zx, yz;
    int i, j, k, i_, j_, k_, iCorrespondence, iNormal;
    float *uS, *a;
    float uM[3], V3Tmp[3], b[3], z[3];
    MESH::Face *pFace;
    float dR[9], R_[9], U_[9], M3x3Tmp[9];
    float detU;
    float invU[9];
    float dphi[3];
    float th;

    for (k = 0; k < nIterations; k++)
    {
        RVLNULL3VECTOR(V3Tmp2);

        if (U == NULL)
            RVLNULLMX3X3(invU);

        for (j = 0; j < pConvexHull->iVisibleFaces.n; j++)
        {
            // uS <- normal of the j-th visible face

            pFace = pConvexHull->faces.Element[pConvexHull->iVisibleFaces.Element[j]];

            uS = pFace->N;

            if (piModelNormalArray)
            {
                RVLMULMX3X3TVECT(R, uS, uM);

                maxw = -2.0f;

                for (i = 0; i < piModelNormalArray->n; i++)
                {
                    iNormal = piModelNormalArray->Element[i];

                    a = sampledUnitSphere.Element + 3 * iNormal;

                    w = RVLDOTPRODUCT3(a, uM);

                    if (w > maxw)
                    {
                        maxw = w;

                        iCorrespondence = iNormal;
                    }
                }
            }
            else
                RVLGET_CLOSEST_CONVEX_TEMPLATE_ELEMENT(convexTemplateLUT, uS, R, uM, i_, j_, k_, iCorrespondence);

            faceCorrespondence[j] = iCorrespondence;

            if (mask[iCorrespondence])
            {
                // a <- the closest template vector

                a = sampledUnitSphere.Element + 3 * iCorrespondence;

                // V3Tmp2 <- V3Tmp2 + w * (uM x a), where w = the area of the j-th visible face * exp(-||a - uM||^2 / (2 * sigmaR^2))

                RVLDIF3VECTORS(a, uM, V3Tmp);

                w = pFace->Area * exp(-kR * RVLDOTPRODUCT3(V3Tmp, V3Tmp));

                RVLCROSSPRODUCT3(uM, a, V3Tmp);

                RVLSCALE3VECTOR(V3Tmp, w, V3Tmp);

                RVLSUM3VECTORS(V3Tmp2, V3Tmp, V3Tmp2);

                if (U == NULL)
                {
                    xx = w * uS[0] * uS[0];
                    yy = w * uS[1] * uS[1];
                    zz = w * uS[2] * uS[2];
                    xy = w * uS[0] * uS[1];
                    yz = w * uS[1] * uS[2];
                    zx = w * uS[2] * uS[0];

                    RVLMXEL(M3x3Tmp, 3, 0, 0) = -(yy + zz);
                    RVLMXEL(M3x3Tmp, 3, 0, 1) = xy;
                    RVLMXEL(M3x3Tmp, 3, 0, 2) = -zx;
                    RVLMXEL(M3x3Tmp, 3, 1, 1) = -(zz + xx);
                    RVLMXEL(M3x3Tmp, 3, 1, 2) = yz;
                    RVLMXEL(M3x3Tmp, 3, 2, 2) = -(xx + yy);

                    RVLSUMMX3X3UT(invU, M3x3Tmp, invU);
                }
            }
        }

        // Only for debugging purpose!

        // if (pConvexHull->iVisibleFaces.n >= 31)
        //	if (RVLABS(sampledUnitSphere.Element[3 * faceCorrespondence[19] + 2]) < 1e-3 &&
        //		RVLABS(sampledUnitSphere.Element[3 * faceCorrespondence[22] + 2]) < 1e-3 &&
        //		RVLABS(sampledUnitSphere.Element[3 * faceCorrespondence[25] + 2]) < 1e-3 &&
        //		RVLABS(sampledUnitSphere.Element[3 * faceCorrespondence[26] + 2]) < 1e-3 &&
        //		RVLABS(sampledUnitSphere.Element[3 * faceCorrespondence[31] + 2]) < 1e-3 ||
        //		RVLABS(sampledUnitSphere.Element[3 * faceCorrespondence[19] + 0]) < 1e-3 &&
        //		RVLABS(sampledUnitSphere.Element[3 * faceCorrespondence[22] + 0]) < 1e-3 &&
        //		RVLABS(sampledUnitSphere.Element[3 * faceCorrespondence[25] + 0]) < 1e-3 &&
        //		RVLABS(sampledUnitSphere.Element[3 * faceCorrespondence[26] + 0]) < 1e-3 &&
        //		RVLABS(sampledUnitSphere.Element[3 * faceCorrespondence[31] + 0]) < 1e-3 ||
        //		RVLABS(sampledUnitSphere.Element[3 * faceCorrespondence[19] + 1]) < 1e-3 &&
        //		RVLABS(sampledUnitSphere.Element[3 * faceCorrespondence[22] + 1]) < 1e-3 &&
        //		RVLABS(sampledUnitSphere.Element[3 * faceCorrespondence[25] + 1]) < 1e-3 &&
        //		RVLABS(sampledUnitSphere.Element[3 * faceCorrespondence[26] + 1]) < 1e-3 &&
        //		RVLABS(sampledUnitSphere.Element[3 * faceCorrespondence[31] + 1]) < 1e-3)
        //		int debug = 0;

        // if (pConvexHull->iVisibleFaces.n >= 19)
        //	if (faceCorrespondence[1] % 11 == 0 &&
        //		faceCorrespondence[6] % 11 == 0 &&
        //		faceCorrespondence[10] % 11 == 0 &&
        //		faceCorrespondence[11] % 11 == 0 &&
        //		faceCorrespondence[13] % 11 == 0)
        //		int debug = 0;

        // if (pConvexHull->iVisibleFaces.n >= 12)
        //	if (faceCorrespondence[0] % 11 == 0 &&
        //		faceCorrespondence[1] % 11 == 0 &&
        //		faceCorrespondence[2] % 11 == 0 &&
        //		faceCorrespondence[11] % 11 == 0)
        //		int debug = 0;

        ///

        // b <- R * V3Tmp2

        RVLMULMX3X3VECT(R, V3Tmp2, b);

        // dphi = -inv(U) * b

        if (U == NULL)
        {
            RVLINVCOV3(invU, U_, detU);

            RVLCOMPLETESIMMX3(U_);

            RVLMULMX3X3VECT(U_, b, dphi);
        }
        else
        {
            RVLMULMX3X3VECT(U, b, dphi);
        }

        RVLNEGVECT3(dphi, dphi);

        // Only for debugging purpose!

        // float E = 0.0f;

        // float e;

        // for (j = 0; j < pConvexHull->iVisibleFaces.n; j++)
        //{
        //	pFace = pConvexHull->faces.Element[pConvexHull->iVisibleFaces.Element[j]];

        //	uS = pFace->N;

        //	RVLMULMX3X3TVECT(R, uS, uM);

        //	a = sampledUnitSphere.Element + 3 * faceCorrespondence[j];

        //	RVLDIF3VECTORS(uM, a, V3Tmp);

        //	e = RVLDOTPRODUCT3(V3Tmp, V3Tmp);

        //	E += (pFace->Area * e);
        //}

        ///

        // dphi = || dphi ||

        th = sqrt(RVLDOTPRODUCT3(dphi, dphi));

        if (RVLABS(th) >= 1e-8)
        {
            // z <- dphi / || dphi ||

            RVLSCALE3VECTOR2(dphi, th, z);

            // dR <- Rot(z, th) (angle axis to rotation matrix)

            AngleAxisToRot<float>(z, th, dR);

            // R <- dR' * R

            RVLMXMUL3X3T1(dR, R, R_);

            RVLCOPYMX3X3(R_, R);
        }
        else
            break;

        // Only for debugging purpose!

        // E = 0.0f;
        // float E2 = 0.0f;
        // float E3 = 0.0f;

        // for (j = 0; j < pConvexHull->iVisibleFaces.n; j++)
        //{
        //	pFace = pConvexHull->faces.Element[pConvexHull->iVisibleFaces.Element[j]];

        //	uS = pFace->N;

        //	// e <- || R' * uS - a ||^2 = || (dR' * RInit)' * uS - a ||^2 = || RInit' * dR * uS - a ||^2 = || RInit * a - dR * uS ||^2

        //	RVLMULMX3X3TVECT(R, uS, uM);

        //	a = sampledUnitSphere.Element + 3 * faceCorrespondence[j];

        //	if (j == 31)
        //	{
        //		if (RVLABS(a[2]) < 1e-3)
        //			int debug = 0;
        //	}

        //	RVLDIF3VECTORS(uM, a, V3Tmp);

        //	e = RVLDOTPRODUCT3(V3Tmp, V3Tmp);

        //	E += (pFace->Area * e);

        //	//// e <- || RInit * a - uS - dphi x uS ||^2

        //	//RVLMULMX3X3VECT(RInit, a, V3Tmp);

        //	//RVLDIF3VECTORS(V3Tmp, uS, V3Tmp);

        //	//RVLCROSSPRODUCT3(dphi, uS, V3Tmp2);

        //	//RVLDIF3VECTORS(V3Tmp, V3Tmp2, V3Tmp);

        //	//e = RVLDOTPRODUCT3(V3Tmp, V3Tmp);

        //	//// E2 <- E2 + e

        //	//E2 += (pFace->Area * e);

        //	//// e <- || Rinit * a - dR * uS ||^2

        //	//RVLMULMX3X3VECT(RInit, a, V3Tmp);

        //	//RVLMULMX3X3VECT(dR, uS, V3Tmp2);

        //	//RVLDIF3VECTORS(V3Tmp, V3Tmp2, V3Tmp);

        //	//e = RVLDOTPRODUCT3(V3Tmp, V3Tmp);

        //	//// E2 <- E2 + e

        //	//E3 += (pFace->Area * e);
        //}

        // int debug = 0;

        ///
    } // for (k = 0; k < nIterations; k++)

    // Compute the fitting score.

    float score = 0.0f;

    for (j = 0; j < pConvexHull->iVisibleFaces.n; j++)
    {
        pFace = pConvexHull->faces.Element[pConvexHull->iVisibleFaces.Element[j]];

        uS = pFace->N;

        if (piModelNormalArray)
        {
            RVLMULMX3X3TVECT(R, uS, uM);

            maxw = -2.0f;

            for (i = 0; i < piModelNormalArray->n; i++)
            {
                iNormal = piModelNormalArray->Element[i];

                a = sampledUnitSphere.Element + 3 * iNormal;

                w = RVLDOTPRODUCT3(a, uM);

                if (w > maxw)
                {
                    maxw = w;

                    iCorrespondence = iNormal;
                }
            }
        }
        else
            RVLGET_CLOSEST_CONVEX_TEMPLATE_ELEMENT(convexTemplateLUT, uS, R, uM, i_, j_, k_, iCorrespondence);

        faceCorrespondence[j] = iCorrespondence;

        if (mask[iCorrespondence])
        {
            a = sampledUnitSphere.Element + 3 * iCorrespondence;

            RVLDIF3VECTORS(a, uM, V3Tmp);

            w = pFace->Area * exp(-kR * RVLDOTPRODUCT3(V3Tmp, V3Tmp));

            score += w;
        }
    }

    return score;
}

void VNClassifier::InitNormalAlignment(
    Array<MESH::Face *> faces,
    Array<int> iFaceArray,
    float *U)
{
    float invU[9];

    RVLNULLMX3X3(invU);

    int i;
    float M3x3Tmp[9];
    float area;
    float *u;
    float xx, yy, zz, xy, zx, yz;
    MESH::Face *pFace;

    for (i = 0; i < iFaceArray.n; i++)
    {
        pFace = faces.Element[iFaceArray.Element[i]];

        u = pFace->N;

        area = pFace->Area;

        xx = area * u[0] * u[0];
        yy = area * u[1] * u[1];
        zz = area * u[2] * u[2];
        xy = area * u[0] * u[1];
        yz = area * u[1] * u[2];
        zx = area * u[2] * u[0];

        RVLMXEL(M3x3Tmp, 3, 0, 0) = -(yy + zz);
        RVLMXEL(M3x3Tmp, 3, 0, 1) = xy;
        RVLMXEL(M3x3Tmp, 3, 0, 2) = -zx;
        RVLMXEL(M3x3Tmp, 3, 1, 1) = -(zz + xx);
        RVLMXEL(M3x3Tmp, 3, 1, 2) = yz;
        RVLMXEL(M3x3Tmp, 3, 2, 2) = -(xx + yy);

        RVLSUMMX3X3UT(invU, M3x3Tmp, invU);
    }

    float detU;

    RVLINVCOV3(invU, U, detU);

    RVLCOMPLETESIMMX3(U);
}

float VNClassifier::Align(
    Array<MESH::Face *> faceArray,
    Array<MESH::Face *> refFaceArray,
    float *R)
{
    float sigTh = 6.0f * DEG2RAD;

    int nIterations = 2;

    float varTh = sigTh * sigTh;

    float csThr = cos(3.0f * sigTh);

    int iFace;

    float totalRefArea = 0.0f;

    for (iFace = 0; iFace < refFaceArray.n; iFace++)
        totalRefArea += refFaceArray.Element[iFace]->Area;

    Array<int> iFaceArray;

    iFaceArray.Element = new int[faceArray.n];

    float *area = new float[faceArray.n];

    float totalArea = 0.0f;

    for (iFace = 0; iFace < faceArray.n; iFace++)
    {
        area[iFace] = faceArray.Element[iFace]->Area;

        totalArea += area[iFace];
    }

    int *iCorresp = new int[faceArray.n];

    float *uM_ = new float[3 * faceArray.n];

    float bestScore = 0.0f;

    int j, k, iR, iFace_, iCorresp_;
    float cs, maxcs, w, e, th, score;
    float *RInit, *uS, *uM, *a;
    float U[9], R_[9], R__[9], V3Tmp[3], V3Tmp2[3], b[3], dphi[3], z[3], dR[9];
    MESH::Face *pFace, *pFace_;

    for (iR = 0; iR < SO3Samples.c; iR++)
    {
        RInit = SO3Samples.Element + 9 * iR;

        RVLCOPYMX3X3(RInit, R_);

        for (k = 0; k <= nIterations; k++)
        {
            iFaceArray.n = 0;

            score = 0.0f;

            for (iFace = 0; iFace < faceArray.n; iFace++)
            {
                pFace = faceArray.Element[iFace];

                // uS <- normal of the j-th visible face

                uS = pFace->N;

                // uM <- normal of the j-th visible face transformed by R_

                uM = uM_ + 3 * iFace;

                RVLMULMX3X3VECT(R_, uS, uM);

                maxcs = csThr;

                iCorresp_ = -1;

                w = 0.0f;

                for (iFace_ = 0; iFace_ < refFaceArray.n; iFace_++)
                {
                    pFace_ = refFaceArray.Element[iFace_];

                    cs = RVLDOTPRODUCT3(uM, pFace_->N);

                    if (cs >= csThr)
                    {
                        e = acos(maxcs);

                        e *= e;

                        w += (pFace_->Area * exp(-e / varTh));

                        if (cs > maxcs)
                        {
                            maxcs = cs;

                            iCorresp_ = iFace_;
                        }
                    }
                }

                iCorresp[iFace] = iCorresp_;

                if (iCorresp_ >= 0)
                {
                    iFaceArray.Element[iFaceArray.n++] = iFace;

                    pFace->Area = area[iFace] * w;

                    score += pFace->Area;
                }
            }

            // if (iFaceArray.n == 76)
            //	int debug = 0;

            if (k == nIterations)
                break;

            InitNormalAlignment(faceArray, iFaceArray, U);

            RVLNULL3VECTOR(V3Tmp2);

            for (j = 0; j < iFaceArray.n; j++)
            {
                iFace = iFaceArray.Element[j];

                pFace = faceArray.Element[iFace];

                uS = pFace->N;

                // uM <- normal of the j-th visible face transformed by R_

                uM = uM_ + 3 * iFace;

                // a <- the closest reference vector

                pFace_ = refFaceArray.Element[iCorresp[iFace]];

                a = pFace_->N;

                // V3Tmp2 <- V3Tmp2 + w * (uM x a), where w = the area of the j-th face

                RVLCROSSPRODUCT3(uM, a, V3Tmp);

                RVLSCALE3VECTOR(V3Tmp, pFace->Area, V3Tmp);

                RVLSUM3VECTORS(V3Tmp2, V3Tmp, V3Tmp2);
            }

            // b <- R_ * V3Tmp2

            RVLMULMX3X3TVECT(R_, V3Tmp2, b);

            // dphi = -inv(U) * b

            RVLMULMX3X3VECT(U, b, dphi);

            RVLNEGVECT3(dphi, dphi);

            // Only for debugging purpose!

            // float E = 0.0f;

            // float e;

            // for (j = 0; j < pConvexHull->iVisibleFaces.n; j++)
            //{
            //	pFace = pConvexHull->faces.Element[pConvexHull->iVisibleFaces.Element[j]];

            //	uS = pFace->N;

            //	RVLMULMX3X3TVECT(R, uS, uM);

            //	a = sampledUnitSphere.Element + 3 * faceCorrespondence[j];

            //	RVLDIF3VECTORS(uM, a, V3Tmp);

            //	e = RVLDOTPRODUCT3(V3Tmp, V3Tmp);

            //	E += (pFace->Area * e);
            //}

            ///

            // dphi = || dphi ||

            th = sqrt(RVLDOTPRODUCT3(dphi, dphi));

            if (RVLABS(th) >= 1e-8)
            {
                // z <- dphi / || dphi ||

                RVLSCALE3VECTOR2(dphi, th, z);

                // dR <- Rot(z, th) (angle axis to rotation matrix)

                AngleAxisToRot<float>(z, th, dR);

                // R_ <- dR' * R_

                RVLMXMUL3X3T1(dR, R_, R__);

                RVLCOPYMX3X3(R__, R_);
            }
            else
                break;
        } // for (k = 0; k < nIterations; k++)

        if (score > bestScore)
        {
            bestScore = score;

            RVLCOPYMX3X3(R_, R);
        }
    } // for (iR = 0; iR < SO3Samples.c; iR++)

    for (iFace = 0; iFace < faceArray.n; iFace++)
        faceArray.Element[iFace]->Area = area[iFace];

    delete[] iFaceArray.Element;
    delete[] area;
    delete[] iCorresp;
    delete[] uM_;

    return bestScore / (totalArea * totalRefArea);
}

void VNClassifier::CanonicalOrientation(float *RCanonical)
{
    int *faceCorrespondence = new int[convexHull.iVisibleFaces.n];

    bool *faceMask = new bool[sampledUnitSphere.h];

    Array<int> iFaceArray;

    iFaceArray.Element = new int[convexHull.iVisibleFaces.n];

    iFaceArray.n = 0;

    Array<int> iModelNormalArray;
    iModelNormalArray.Element = NULL;

    int i;
    float U[9];
    float *U_;
    Array<int> *piModelNormalArray;

    if (bAlignWithConvexTemplate)
    {
        for (i = 0; i < sampledUnitSphere.h; i++)
            faceMask[i] = true;

        InitNormalAlignment(convexHull.faces, convexHull.iVisibleFaces, U);

        U_ = U;

        piModelNormalArray = NULL;
    }
    else
    {
        iModelNormalArray.n = 6;

        iModelNormalArray.Element = new int[iModelNormalArray.n];

        memset(faceMask, 0, sampledUnitSphere.h * sizeof(bool));

        for (i = 0; i < iModelNormalArray.n; i++)
        {
            iModelNormalArray.Element[i] = i * 11;

            faceMask[i * 11] = true;
        }

        U_ = NULL;

        piModelNormalArray = &iModelNormalArray;
    }

    float RXZ[9];
    float *X = RXZ;
    float *Y = RXZ + 3;
    float *Z = RXZ + 6;

    CRVLTimer timer;

    timer.Start();

    float dPan = 2.0f * PI / (float)(CTIDescriptorMapingData.nPan);
    float dTilt = 0.25f * PI / (float)(CTIDescriptorMapingData.nTilt);
    float dRoll = 0.5f * PI / (float)(CTIDescriptorMapingData.nRoll);

    int nRollTotal = 4 * CTIDescriptorMapingData.nRoll;
    int nZ = CTIDescriptorMapingData.nPan * (CTIDescriptorMapingData.nTilt - 1) + 1;
    int nDescriptorsPerBlock = 6 * nRollTotal;
    int totalBlockDescriptorSize = nDescriptorsPerBlock * sampledUnitSphere.h;

    int rollBlockSize = CTIDescriptorMapingData.nRoll * sampledUnitSphere.h;

    memset(f, 0, nZ * totalBlockDescriptorSize * sizeof(float));

    float Z_[3];

    Z_[2] = 1.0f;

    float X_[3];

    X_[2] = 0.0f;

    float fmaxiTilt = (float)(CTIDescriptorMapingData.nTilt - 1);

    int iZ = 0;

    // Only for debugging purpose!

    // float *ZArchive = new float[3 * nZ];

    ///

    float maxScore = 0.0f;

    float *V3Tmp;
    int iPan, iTilt, iRoll, iRollBlockStart, nPan, iZRef, iRollRef;
    float pan, tilt, roll, maxTilt, cPan, sPan, cTilt, sTilt, cRoll, sRoll, d0, d_, dmax, abscPan, abssPan, x, y, fTmp;
    float RRoll[9], R0[9], RSM[9];
    int j;
    float *f_;
    float score;

    for (iTilt = 0; iTilt < CTIDescriptorMapingData.nTilt; iTilt++)
    {
        nPan = (iTilt > 0 ? CTIDescriptorMapingData.nPan : 1);

        for (iPan = 0; iPan < nPan; iPan++)
        {
            pan = (float)iPan * dPan;

            cPan = cos(pan);
            sPan = sin(pan);

            abscPan = RVLABS(cPan);
            abssPan = RVLABS(sPan);

            fTmp = (abscPan >= abssPan ? abscPan : abssPan);

            Z_[0] = cPan / fTmp;
            Z_[1] = sPan / fTmp;

            fTmp = sqrt(RVLDOTPRODUCT3(Z_, Z_));

            maxTilt = acos(1.0f / fTmp);

            tilt = maxTilt / fmaxiTilt * (float)iTilt;

            cTilt = cos(tilt);
            sTilt = sin(tilt);

            Z[0] = cPan * sTilt;
            Z[1] = sPan * sTilt;
            Z[2] = cTilt;

            // Only for debugging purpose!

            // float *ZArchive_ = ZArchive + 3 * iZ;

            // RVLCOPY3VECTOR(Z, ZArchive_);

            ///

            X_[0] = -Z[1];
            X_[1] = Z[0];

            fTmp = sqrt(RVLDOTPRODUCT3(X_, X_));

            if (fTmp >= 1e-10)
            {
                RVLSCALE3VECTOR2(X_, fTmp, X);
            }
            else
                RVLSET3VECTOR(X, 1.0f, 0.0f, 0.0f);

            RVLCROSSPRODUCT3(Z, X, Y);

            iRollBlockStart = iZ * totalBlockDescriptorSize;

            // if (iZ == 8)
            //	int debug = 0;

            for (iRoll = 0; iRoll < CTIDescriptorMapingData.nRoll; iRoll++)
            {
                roll = (float)iRoll * dRoll;

                cRoll = cos(roll);
                sRoll = sin(roll);

                RVLROTZ(cRoll, sRoll, RRoll);

                RVLMXMUL3X3(RRoll, RXZ, RSM);

                RVLCOPYMX3X3T(RSM, R0);

                score = FitCTI(&convexHull, U_, R0, faceMask, 2, R + 9 * (iZ * CTIDescriptorMapingData.nRoll + iRoll), faceCorrespondence, piModelNormalArray);

#ifdef RVLVN_CLASSIFY_CTI_VERSION1
                f_ = f + iRollBlockStart + iRoll * sampledUnitSphere.h;

                for (j = 0; j < convexHull.iVisibleFaces.n; j++)
                    f_[faceCorrespondence[j]] += convexHull.faces.Element[convexHull.iVisibleFaces.Element[j]]->Area;
#endif
                if (score > maxScore)
                {
                    maxScore = score;

                    iZRef = iZ;
                    iRollRef = iRoll;
                }
            }

            iZ++;
        } // for (iPan = 0; iPan < nPan; iPan++)
    }     // for (iTilt = 0; iTilt < CTIDescriptorMapingData.nTilt; iTilt++)

    RVL_DELETE_ARRAY(iModelNormalArray.Element);

    // Only for debugging purpose!

    // FILE *fpZArchive = fopen("C:\\RVL\\Debug\\Z.txt", "w");

    // PrintMatrix<float>(fpZArchive, ZArchive, nZ, 3);

    // fclose(fpZArchive);

    // delete[] ZArchive;

    ///

    int k, iConvexTempleteElement, i_, j_, k_;
    float *R__, *RXZ_, *uS;
    float R_[9], RMS_[9], uM[3];
    float *N;
    float N_[3], absProj[3];
    int imax;

    // maxScore = 0.0f;

    // for (iZ = 0; iZ < nZ; iZ++)
    //{
    //	for (iRoll = 0; iRoll < CTIDescriptorMapingData.nRoll; iRoll++)
    //	{
    //		R__ = R + 9 * (iZ * CTIDescriptorMapingData.nRoll + iRoll);

    //		score = 0.0f;

    //		for (i = 0; i < convexHull.iVisibleFaces.n; i++)
    //		{
    //			iFace = convexHull.iVisibleFaces.Element[i];

    //			pFace = convexHull.faces.Element[iFace];

    //			if (pFace->Area > 0.0f)
    //			{
    //				N = pFace->N;

    //				RVLMULMX3X3TVECT(R__, N, N_);

    //				absProj[0] = RVLABS(N_[0]);
    //				absProj[1] = RVLABS(N_[1]);
    //				absProj[2] = RVLABS(N_[2]);

    //				imax = (absProj[0] >= absProj[1] ? 0 : 1);
    //				if (absProj[2] > absProj[imax])
    //					imax = 2;

    //				score += (pFace->Area * absProj[imax]);
    //			}
    //		}

    //		if (score > maxScore)
    //		{
    //			maxScore = score;

    //			iZRef = iZ;
    //			iRollRef = iRoll;
    //		}
    //	}
    //}

    R__ = R + 9 * (iZRef * CTIDescriptorMapingData.nRoll + iRollRef);

    // roll = (float)iRollRef * dRoll;

    // cRoll = cos(roll);
    // sRoll = sin(roll);

    // RVLROTZ(cRoll, sRoll, RRoll);

    // RXZ_ = CTIDescriptorMapingData.R;

    // RVLMXMUL3X3T2(RRoll, RXZ_, R_);

    // RVLMXMUL3X3(R__, R_, RMS_);

    RVLCOPYMX3X3(R__, RMS_);

    if (bAlignWithConvexTemplate)
    {
        iFaceArray.n = 0;

        int iFace;
        MESH::Face *pFace;

        for (j = 0; j < convexHull.iVisibleFaces.n; j++)
        {
            iFace = convexHull.iVisibleFaces.Element[j];

            pFace = convexHull.faces.Element[iFace];

            uS = pFace->N;

            RVLGET_CLOSEST_CONVEX_TEMPLATE_ELEMENT(convexTemplateLUT, uS, RMS_, uM, i_, j_, k_, iConvexTempleteElement);

            if (faceMask[iConvexTempleteElement])
                iFaceArray.Element[iFaceArray.n++] = iFace;
        }

        memset(faceMask, 0, sampledUnitSphere.h * sizeof(bool));

        for (k = 0; k < 6; k++)
            faceMask[k * 11] = true;

        InitNormalAlignment(convexHull.faces, iFaceArray, U);

        FitCTI(&convexHull, U, RMS_, faceMask, 1, RCanonical, faceCorrespondence);
    }
    else
    {
        RVLCOPYMX3X3(RMS_, RCanonical);
    }

    delete[] faceCorrespondence;
    delete[] faceMask;
    delete[] iFaceArray.Element;
    delete[] iModelNormalArray.Element;
}

void VNClassifier::ComputeRotationCostJacobian(
    Mesh *pConvexHull,
    float *RCanonical,
    float *d,
    unsigned char *bd,
    float *J,
    Array<int> *piVertexArray)
{
    // Parameters.

    float th = 0.25f * PI;

    // Constants.

    float cs = cos(th);
    float sn = sin(th);

    //

    float *d_ = new float[sampledUnitSphere.h];
    unsigned char *bd_ = new unsigned char[sampledUnitSphere.h];

    float RM_M[9], RM_[9], t[3], e[2];
    int i, j, iAxis, ne;
    int nV[2];
    float sign, sn_, e2, J_;

    for (iAxis = 0; iAxis < 3; iAxis++)
    {
        sign = 1.0f;

        for (i = 0; i < 2; i++)
        {
            sn_ = sign * sn;

            switch (iAxis)
            {
            case 0:
                RVLROTX(cs, sn_, RM_M);

                break;
            case 1:
                RVLROTY(cs, sn_, RM_M);

                break;
            case 2:
                RVLROTZ(cs, sn_, RM_M);
            }

            RVLMXMUL3X3(RCanonical, RM_M, RM_);

            ComputeDescriptor(pConvexHull, RM_, d_, bd_, piVertexArray);

            // FILE *fp = fopen("C:\\RVL\\ExpRez\\d.txt", "w");

            // PrintMatrix<float>(fp, d, 66, 1);

            // fclose(fp);

            // fp = fopen("C:\\RVL\\ExpRez\\d_.txt", "w");

            // PrintMatrix<float>(fp, d_, 66, 1);

            // fclose(fp);

            e[i] = FitLS(d, bd, d_, bd_, t, nV[i]);

            sign = -1.0f;
        }

        J[iAxis] = sqrt(((float)(nV[0]) * e[0] + (float)(nV[1]) * e[1]) / ((float)(nV[0] + nV[1]))) / th;
    }

    // Free memory.

    delete[] d_;
    delete[] bd_;
}

void VNClassifier::SegmentToParts(Mesh *pMesh, int ID)
{
    if (recognitionMethod == 0)
    {
        // Part segmentation for aligned models.

        float R[9];
        float t[3];

        RVLUNITMX3(R);
        RVLNULL3VECTOR(t);

        float *dS = NULL;
        bool *bdS = NULL;

        Box<float> SBoundingBox;

        VNInstance *pVNSceneInstance;

        if (bLoadSceneVNInstances)
            pVNSceneInstance = sceneVNArray.Element[ID];
        else
            ComputeDescriptor(pMesh, R, t, dS, bdS, SBoundingBox, -1, pMem, &pVNSceneInstance);

        if (bGenerateSceneVNInstances)
        {
            QList<VNInstance> *pSceneVNList = &sceneVNList;
            RVLQLIST_ADD_ENTRY(pSceneVNList, pVNSceneInstance);

            if (bSaveGeneratedVNInstances)
            {
                RVL::SURFEL::Cell *pCell;
                RVL::QLIST::Index *piSurfelIdx;
                RVL::Surfel *pSurfel;
                RVL::QLIST::Index2 *piPtIdx;
                int nPoints = pMesh->NodeArray.n;
                int *pointCellID = new int[nPoints];
                memset(pointCellID, 0xFF, nPoints * sizeof(int));

                for (int iCell = 0; iCell < pVNSceneInstance->surfelCells.n; iCell++)
                {
                    pCell = pVNSceneInstance->surfelCells.Element + iCell;

                    piSurfelIdx = pCell->surfelList.pFirst;

                    while (piSurfelIdx)
                    {
                        pSurfel = pSurfels->NodeArray.Element + piSurfelIdx->Idx;

                        piPtIdx = pSurfel->PtList.pFirst;

                        while (piPtIdx)
                        {
                            pointCellID[piPtIdx->Idx] = iCell;

                            piPtIdx = piPtIdx->pNext;
                        }

                        piSurfelIdx = piSurfelIdx->pNext;
                    }
                }

                // FILE *fpPointCellID = fopen(ScenePointCellID, "w");

                // FILE *fpPointCellID = fopen(std::string(ScenePointCellID) + std::string(), "w");
                // for (int iPoint = 0; iPoint < nPoints; iPoint++)
                //{
                //	fprintf(fpPointCellID, "%d\n", pointCellID[iPoint]);
                // }
                // fclose(fpPointCellID);

                delete[] pointCellID;
            }
        }

        pVNSceneInstance->ProjectToLatentSubspace(this);

        VN_::PartAssociation *componentAssociation, *cellAssociation;

        VN_::CTINetOutput CTINet;

        pVNSceneInstance->CTINet(this, 0, CTINet, componentAssociation, cellAssociation);

        // pVNSceneInstance->NearestNeighborsAssociation(this, 0, association);

        // pVNSceneInstance->ProbabilisticAssociation(this, 0, association);

        if (bVisualizePartAssociation && bGenerateSceneVNInstances)
        {
            uchar SelectionColor[] = {0, 255, 0};

            InitDisplay(visualizationData.pVisualizer, pMesh, SelectionColor);

            visualizationData.pVNSceneInstance = pVNSceneInstance;

            visualizationData.componentAssociation = componentAssociation;
            visualizationData.cellAssociation = cellAssociation;

            pVNSceneInstance->VisualizePartAssociation(pMesh, this, 0, componentAssociation, cellAssociation, -1, 1);
        }

        delete[] componentAssociation;
        delete[] cellAssociation;

        // VNClass VNClass;
        // VNClassifier *pClassifier = this;
        // FILE *fp = fopen((std::string(pClassifier->resultsFolder) + "\\VNClasses.dat").data(), "rb");
        // VNClass.Load(pClassifier, fp);

        // pVNSceneInstance->Match2(VNClass, pClassifier);

        // pVNSceneInstance->MatchComponents(&modelVNList, Correspondences, this, pMesh);

        // delete[] Correspondences;
    }
}

void VNClassifier::SetSceneFileName(char *sceneFileName_)
{
    RVLCopyString(sceneFileName_, &sceneFileName);
}

void VNClassifier::SaveDescriptor(
    FILE *fp,
    float *d,
    bool *bd,
    int iModel,
    int iMetaModel)
{
    VN *pModel = models[iMetaModel];

    fprintf(fp, "%d\t%d\t", iModel, iMetaModel);

    int i;

    for (i = 0; i < pModel->featureArray.n; i++)
        fprintf(fp, "%f\t", d[i]);

    for (i = 0; i < pModel->featureArray.n; i++)
        fprintf(fp, "%d\t", (int)(bd[i]));

    fprintf(fp, "\n");
}

// #define RVLVN_SAVEDESCRIPTOR_ALL_VALID

void VNClassifier::SaveDescriptor(
    FILE *fp,
    float *d,
    uchar *bd,
    float *R,
    int iModel,
    int iPart,
    int iCluster,
    int type)
{
    // Curent implementation of Matlab function RVLPSGMFormat - 20.2.2019.
    /*
    iModel = 1;
    iPart = iModel + 1;
    iSegment = iPart + 1;
    iType = iSegment + 1;
    iRot = iType + 1;
    it = iRot + 9;
    id = it + 3;
    iValid = id + 66;
    */

    float t[3];

    RVLNULL3VECTOR(t);

#ifdef RVLCTIDESCRIPTOR_SAVE_IN_OLD_FORMAT
    fprintf(fp, "%d\t%d\t", iModel, iCluster); // old format
#else
    fprintf(fp, "%d\t%d\t%d\t%d\t", iModel, iPart, iCluster, type);
#endif

    int i;

    for (i = 0; i < 9; i++)
        fprintf(fp, "%f\t", R[i]);

    for (i = 0; i < 3; i++)
        fprintf(fp, "%f\t", t[i]);

    for (i = 0; i < sampledUnitSphere.h; i++)
        fprintf(fp, "%f\t", trainingModelScale * d[i]);

#ifdef RVLVN_SAVEDESCRIPTOR_ALL_VALID
    for (i = 0; i < sampledUnitSphere.h; i++)
        fprintf(fp, "%d\t", 1);
#else
    for (i = 0; i < sampledUnitSphere.h; i++)
        fprintf(fp, "%d\t", (int)(bd[i]));
#endif

    for (i = 0; i < sampledUnitSphere.h; i++)
        fprintf(fp, "%f\t", 0.0f);

    for (i = 0; i < 3; i++)
        fprintf(fp, "%f\t", 0.0f);

    fprintf(fp, "\n");
}

// Vidovic - commented on 3.10.2018 - CHECK with Cupec
/*void VNClassifier::SaveDescriptor(
    FILE *fp,
    float *d,
    uchar *bd,
    int iModel,
    int iMetaModel)
{
    VN *pModel = models[iMetaModel];

    fprintf(fp, "%d\t%d\t", iModel, iMetaModel);

    int i;

    for (i = 0; i < pModel->featureArray.n; i++)
        fprintf(fp, "%f\t", d[i]);

    for (i = 0; i < pModel->featureArray.n; i++)
        fprintf(fp, "%d\t", (int)(bd[i]));

    fprintf(fp, "\n");
}*/

void VNClassifier::SaveDescriptors(
    FILE *fp,
    float *d,
    uchar *bd,
    int iCluster,
    int type)
{
    float t[3];

    RVLNULL3VECTOR(t);

    float *d_ = d;
    uchar *bd_ = bd;
    float *R = SO3Samples.Element;
    float *R_ = R;

    int i, j;

    for (j = 0; j < SO3Samples.c; j++, d_ += sampledUnitSphere.h, bd_ += sampledUnitSphere.h, R_ += 9)
    {
        fprintf(fp, "%d\t%d\t", type, iCluster);

        for (i = 0; i < 9; i++)
            fprintf(fp, "%f\t", R_[i]);

        for (i = 0; i < 3; i++)
            fprintf(fp, "%f\t", t[i]);

        for (i = 0; i < sampledUnitSphere.h; i++)
            fprintf(fp, "%f\t", d_[i]);

        for (i = 0; i < sampledUnitSphere.h; i++)
            fprintf(fp, "%d\t", (int)(bd_[i]));

        for (i = 0; i < sampledUnitSphere.h; i++)
            fprintf(fp, "%f\t", 0.0f);

        for (i = 0; i < 3; i++)
            fprintf(fp, "%f\t", 0.0f);

        fprintf(fp, "\n");
    }
}

void VNClassifier::SaveDescriptors(
    FILE *fp,
    float *d,
    uchar *bd,
    int nDescriptors,
    int iCluster,
    int type)
{
    float t[3];

    RVLNULL3VECTOR(t);

    float R[9];

    RVLUNITMX3(R)

    float *d_ = d;
    uchar *bd_ = bd;

    int i, j;

    for (j = 0; j < nDescriptors; j++, d_ += sampledUnitSphere.h, bd_ += sampledUnitSphere.h)
    {
        fprintf(fp, "%d\t%d\t", type, iCluster);

        for (i = 0; i < 9; i++)
            fprintf(fp, "%f\t", R[i]);

        for (i = 0; i < 3; i++)
            fprintf(fp, "%f\t", t[i]);

        for (i = 0; i < sampledUnitSphere.h; i++)
            fprintf(fp, "%f\t", d_[i]);

        for (i = 0; i < sampledUnitSphere.h; i++)
            fprintf(fp, "%d\t", (int)(bd_[i]));

        for (i = 0; i < sampledUnitSphere.h; i++)
            fprintf(fp, "%f\t", 0.0f);

        for (i = 0; i < 3; i++)
            fprintf(fp, "%f\t", 0.0f);

        fprintf(fp, "\n");
    }
}

void VNClassifier::SaveVertices(
    FILE *fp,
    RECOG::PSGM_::Cluster *pSCluster,
    int iCluster,
    int clusterType)
{
    int iVertex, nVertices;
    nVertices = pSCluster->iVertexArray.n;

    SURFEL::Vertex *pVertex;

    int iClusterAll;

    if (clusterType == RVLVN_CLUSTER_TYPE_CONVEX)
        iClusterAll = iConvexCluster;
    else if (clusterType == RVLVN_CLUSTER_TYPE_CONCAVE)
        iClusterAll = iConcaveCluster;

    char *fileName = NULL;

    RVLCopyString(sceneFileName, &fileName);

    fileName[strlen(fileName) - 4] = 0;

    fprintf(fp, "%d\t%s\t%d\t%d\n", iClusterAll, fileName, iCluster, pSCluster->iVertexArray.n);

    for (iVertex = 0; iVertex < pSCluster->iVertexArray.n; iVertex++)
    {
        pVertex = pSurfels->vertexArray.Element[pSCluster->iVertexArray.Element[iVertex]];

        fprintf(fp, "%f\t%f\t%f\n", pVertex->P[0], pVertex->P[1], pVertex->P[2]);
    }

    delete[] fileName;
}

void VNClassifier::SaveLatentVector(
    FILE *fp,
    float *q,
    int iClass)
{
    fprintf(fp, "%d\t", iClass);

    ClassData *pClass = classArray.Element + iClass;

    PrintMatrix(fp, q, 1, pClass->M.h);
}

void VNClassifier::LoadDescriptor(
    FILE *fp,
    float *&d,
    bool *&bd,
    int &iModel,
    int &iMetaModel)
{
    fscanf(fp, "%d\t%d\t", &iModel, &iMetaModel);

    VN *pModel = models[iMetaModel];

    d = new float[pModel->featureArray.n];

    int i;

    for (i = 0; i < pModel->featureArray.n; i++)
        fscanf(fp, "%f\t", d + i);

    bd = new bool[pModel->featureArray.n];

    int ibd;

    for (i = 0; i < pModel->featureArray.n; i++)
    {
        fscanf(fp, "%d\t", &ibd);

        bd[i] = (bool)ibd;
    }
}

void VNClassifier::LoadLatentVector(
    FILE *fp,
    float *&q,
    int &iClass)
{
    fscanf(fp, "%d\t", &iClass);

    ClassData *pClass = classArray.Element + iClass;

    q = new float[pClass->M.h];

    int i;

    for (i = 0; i < pClass->M.h; i++)
        fscanf(fp, "%f\t", q + i);
}

void VNClassifier::LoadShapeSpace(char *shapeSpaceFileName)
{
    FILE *fp = fopen(shapeSpaceFileName, "r");

    if (fp == NULL)
        return;

    bShapeSpace = true;

    char shapeSpaceFilePath[500];

    strcpy(shapeSpaceFilePath, shapeSpaceFileName);

    char *pFileName = strrchr(shapeSpaceFilePath, '\\') + 1;

    int iFeature, i, j, l;
    char line[500];
    ClassData *pClass;
    int iClass;
    char shapeSpaceFileName_[500];
    FILE *fp_;
    VN *pModel;
    float *N, *M_, *Q;

    while (true)
    {
        fgets(line, 500, fp);

        if (sscanf(line, "%d\t%s\n", &iClass, shapeSpaceFileName_) == 2)
        {
            if (iClass >= 0 && iClass < classArray.n)
            {
                pClass = classArray.Element + iClass;

                strcpy(pFileName, shapeSpaceFileName_);

                fp_ = fopen(shapeSpaceFilePath, "rb");

                fread(&(pClass->M.w), sizeof(int), 1, fp_);
                fread(&(pClass->M.h), sizeof(int), 1, fp_);
                pClass->M.Element = new float[pClass->M.w * pClass->M.h];
                fread(pClass->M.Element, sizeof(float), pClass->M.w * pClass->M.h, fp_);
                int m = pClass->M.h - 4;
                pClass->qMin = new float[m];
                fread(pClass->qMin, sizeof(float), m, fp_);
                pClass->qMax = new float[m];
                fread(pClass->qMax, sizeof(float), m, fp_);
                fread(&(pClass->nHull), sizeof(int), 1, fp_);
                if (pClass->nHull > 0)
                {
                    pClass->A = new float[pClass->nHull * m];
                    fread(pClass->A, sizeof(float), pClass->nHull * m, fp_);
                    pClass->b = new float[pClass->nHull];
                    fread(pClass->b, sizeof(float), pClass->nHull, fp_);
                }
                else
                {
                    pClass->A = NULL;
                    pClass->b = NULL;
                }
                fread(&(pClass->nq0), sizeof(int), 1, fp_);
                if (pClass->nq0 > 0)
                {
                    pClass->q0 = new float[pClass->nq0 * m];
                    fread(pClass->q0, sizeof(float), pClass->nq0 * m, fp_);
                }
                else
                    pClass->q0 = NULL;
                RVLNULLMX3X3(pClass->T);
                pModel = models[pClass->iMetaModel];
                for (iFeature = 0; iFeature < pModel->featureArray.n; iFeature++)
                {
                    N = pModel->featureArray.Element[iFeature].N;

                    M_ = pClass->M.Element + iFeature;

                    for (l = 0; l < 3; l++)
                        for (j = l; j < 3; j++)
                            pClass->T[l * 3 + j] += (N[j] * M_[l * pModel->featureArray.n]);
                }
                RVLCOMPLETESIMMX3(pClass->T);

                fclose(fp_);

                pClass->Q = new float[pClass->M.h * pClass->M.h * pClass->M.w];

                for (i = 0; i < pClass->M.w; i++)
                {
                    Q = pClass->Q + pClass->M.h * pClass->M.h * i;

                    M_ = pClass->M.Element + i;

                    for (j = 0; j < pClass->M.h; j++)
                        for (l = 0; l < pClass->M.h; l++)
                            Q[l + j * pClass->M.h] = M_[j * pClass->M.w] * M_[l * pClass->M.w];

                    // FILE *fpQ = fopen("D:\\Cupec\\Documents\\Experiments\\Results\\Q.txt", "w");

                    // PrintMatrix<float>(fpQ, Q, pClass->M.h, pClass->M.h);

                    // fclose(fpQ);
                }
            }
        }
        else if (strcmp(line, "end") == 0)
            break;
    }

    fclose(fp);
}

void VNClassifier::InitDisplay(
    Visualizer *pVisualizer,
    Mesh *pMesh,
    unsigned char *selectionColor)
{
    pVisualizer->normalLength = 10.0;

    pVisualizer->SetMesh(pMesh);

    visualizationData.pMesh = pMesh;
    visualizationData.pSurfels = pSurfels;
    visualizationData.pClassifier = this;
    visualizationData.pVisualizer = pVisualizer;
    RVLCOPY3VECTOR(selectionColor, visualizationData.selectionColor);
    visualizationData.iSelectedCluster = -1;
    visualizationData.iSelectedSuperSegment = -1;
    visualizationData.iSelectedComponent = -1;
    visualizationData.ZAxesActor = NULL;
    visualizationData.bHypothesisSelected = false;
    pSurfels->DisplayData.keyPressUserFunction = &RECOG::VN_::keyPressUserFunction;
    pSurfels->DisplayData.mouseRButtonDownUserFunction = &RECOG::VN_::mouseRButtonDownUserFunction;
    pSurfels->DisplayData.vpUserFunctionData = &visualizationData;

    pSurfels->InitDisplay(pVisualizer, pMesh, pSurfelDetector);
}

void VNClassifier::ResetClusterColor(int iCluster)
{
    RECOG::ResetClusterColor(visualizationData.pVisualizer, visualizationData.pMesh, pSurfels, iCluster, SClusters,
                             visualizationData.clusterColor, clusterMap);
}

void VNClassifier::DisplayClusters()
{
    RECOG::DisplayClusters(visualizationData.pVisualizer, SClusters, visualizationData.pMesh, pSurfels, visualizationData.clusterColor,
                           clusterMap);
}

void VNClassifier::DisplaySelectedCluster(int iCluster)
{
    RECOG::DisplaySelectedCluster(visualizationData.pVisualizer, visualizationData.pMesh, pSurfels, iCluster, SClusters,
                                  visualizationData.iSelectedCluster, clusterMap, visualizationData.selectionColor, visualizationData.clusterColor);
}

void VNClassifier::DisplaySelectedSuperSegment(int iSuperSegment)
{
    // Display segments.

    VN_::SuperSegment *pSelectedSuperSegment = superSegments.Element[iSuperSegment];
    Array<int> emptyArray;
    emptyArray.n = 0;
    emptyArray.Element = NULL;
    Array<int> prevSegments = (visualizationData.iSelectedSuperSegment >= 0 ? superSegments.Element[visualizationData.iSelectedSuperSegment]->segments : emptyArray);

    RECOG::DisplaySelectedClusters(visualizationData.pVisualizer, visualizationData.pMesh, pSurfels, pSelectedSuperSegment->segments, SClusters,
                                   prevSegments, clusterMap, visualizationData.selectionColor, visualizationData.clusterColor);

    // Display reference frame.

    visualizationData.pVisualizer->DisplayReferenceFrame(&(pSelectedSuperSegment->pose), 0.05f);
}

void VNClassifier::DisplayInterpretation(
    Mesh *pMesh,
    uchar *ptColor,
    cv::Mat image,
    int pointSize,
    bool bBGR)
{
    uchar *RGB = image.data;

    memset(RGB, 0, 3 * camera.h * camera.w * sizeof(uchar));

    int iPix = 0;

    int u, v, du, dv, u_, v_;
    uchar *RGB_;

    for (v = 0; v < camera.h; v++)
        for (u = 0; u < camera.w; u++, iPix++, RGB += 3)
        {
            RGB_ = pMesh->NodeArray.Element[iPix].RGB;

            if (bBGR)
                RVLSET3VECTOR(RGB, RGB_[2], RGB_[1], RGB_[0])
            else
            {
                RVLCOPY3VECTOR(RGB_, RGB)
            }
        }

    Rect<int> window;
    window.minx = 0;
    window.maxx = camera.w - 1;
    window.miny = 0;
    window.maxy = camera.h - 1;

    int iObject, iPt;
    VN_::Hypothesis2 *pHypothesis;
    Array<OrientedPoint> PC;
    OrientedPoint *pPt;
    float P[3], N[3];
    float *R, *t;

    for (iObject = 0; iObject < interpretation2.size(); iObject++)
    {
        pHypothesis = interpretation2[iObject];

        if (pHypothesis)
        {
            R = pHypothesis->R;
            t = pHypothesis->P;

            PC = pShapeInstanceDetection->sampledModels.Element[pHypothesis->iModel];

            for (iPt = 0; iPt < PC.n; iPt++)
            {
                pPt = PC.Element + iPt;

                RVLTRANSF3(pPt->P, R, t, P);
                RVLMULMX3X3VECT(R, pPt->N, N);

                if (RVLDOTPRODUCT3(P, N) >= 0)
                    continue;

                u = (int)round(camera.fu * P[0] / P[2] + camera.uc);
                v = (int)round(camera.fv * P[1] / P[2] + camera.vc);

                for (du = 0; du < pointSize; du++)
                    for (dv = 0; dv < pointSize; dv++)
                    {
                        u_ = u - pointSize / 2 + du;
                        v_ = v - pointSize / 2 + dv;

                        if (!IsInRect<int>(u_, v_, window))
                            continue;

                        RGB = image.data + 3 * (u_ + v_ * camera.w);

                        if (bBGR)
                            RVLSET3VECTOR(RGB, ptColor[2], ptColor[1], ptColor[0])
                        else
                        {
                            RVLCOPY3VECTOR(ptColor, RGB)
                        }
                    }
            }
        }
    }
}

// Vidovic
void VNClassifier::MergeClusters(
    Mesh *pMesh,
    Array<RECOG::SceneCluster> *pSClusters,
    bool bMergeClusters,
    bool verbose)
{

    printf("Cluster merging started...");

    Init(pMesh);

    RECOG::SceneCluster *pSCluster;
    RECOG::PSGM_::Cluster *pSCluster_;

    int nClusters = pSClusters->n;
    int iCTIPair;

    // create array for storing original and merged clusters
    std::vector<SURFEL::Object> MergedSClusters;
    MergedSClusters.clear();

    // SURFEL::Object *pObjectCluster;
    SURFEL::Object Object;

    // DEBUG
    // nClusters = 5;

    // Copy original clusters to MergedSClusters array
    for (int i = 0; i < nClusters; i++)
    {
        pSCluster = pSClusters->Element + i;

        if (pSCluster->type == RVLVN_CLUSTER_TYPE_CONVEX || pSCluster->type == RVLVN_CLUSTER_TYPE_CONCAVE)
        {
            pSCluster_ = (RECOG::PSGM_::Cluster *)(pSCluster->vpCluster);

            // pObjectCluster = new SURFEL::Object;

            // add cluster to array of merged clusters
            // MergedSClusters.push_back(*pObjectCluster);
            MergedSClusters.push_back(Object);

            // add cluster type flag and valid flag
            bool test;
            unsigned char flags;
            if (pSCluster->type == RVLVN_CLUSTER_TYPE_CONCAVE)
                MergedSClusters[i].flags = (RVLPCSEGMENT_OBJECT_FLAG_CONCAVE | RVLPCSEGMENT_OBJECT_FLAG_VALID);
            else
                MergedSClusters[i].flags = (RVLPCSEGMENT_OBJECT_FLAG_VALID);

            MergedSClusters[i].iClusterArray.Element = new int[nClusters];
            // MergedSClusters[i].iClusterArray.n = 0;
            MergedSClusters[i].iClusterArray.Element[0] = i;
            MergedSClusters[i].iClusterArray.n = 1;

            /*
            //Copy surfel list
            QList<QLIST::Index> *pSurfelList = &MergedSClusters[i].surfelList;
            RVLQLIST_INIT(pSurfelList);

            for (int iSurfel = 0; iSurfel < pSCluster_->iSurfelArray.n; iSurfel++)
            {
            QLIST::Index *pSurfel;

            RVLMEM_ALLOC_STRUCT(pMem, QLIST::Index, pSurfel); //pMem or pMem0??

            pSurfel->Idx = pSCluster_->iSurfelArray.Element[iSurfel];

            RVLQLIST_ADD_ENTRY(pSurfelList, pSurfel);
            }*/

            // Copy surfel array
            MergedSClusters[i].iSurfelArray.Element = new int[pSCluster_->iSurfelArray.n];
            memcpy(MergedSClusters[i].iSurfelArray.Element, pSCluster_->iSurfelArray.Element, sizeof(int) * pSCluster_->iSurfelArray.n);
            MergedSClusters[i].iSurfelArray.n = pSCluster_->iSurfelArray.n;

            // Copy vertex array
            MergedSClusters[i].iVertexArray.Element = new int[pSCluster_->iVertexArray.n];
            memcpy(MergedSClusters[i].iVertexArray.Element, pSCluster_->iVertexArray.Element, sizeof(int) * pSCluster_->iVertexArray.n);
            MergedSClusters[i].iVertexArray.n = pSCluster_->iVertexArray.n;

            // Copy descriptor
            MergedSClusters[i].d = new float[alignment.convexTemplate.n];
            memcpy(MergedSClusters[i].d, pSCluster_->d, sizeof(float) * alignment.convexTemplate.n);

            // Copy descriptor validity
            MergedSClusters[i].bd = new uchar[alignment.convexTemplate.n];
            memcpy(MergedSClusters[i].bd, pSCluster_->bd, sizeof(uchar) * alignment.convexTemplate.n);

            // Copy cluster size
            MergedSClusters[i].size = pSCluster_->size;

            // Copy cluster label
            MergedSClusters[i].label = (pMesh->bLabels ? pSurfels->NodeArray.Element[pSCluster_->iSurfelArray.Element[0]].ObjectID : -1);

            // FILE *fpDebug = fopen("C:\\RVL\\ExpRez\\d.txt", "w");

            // PrintMatrix<float>(fpDebug, MergedSClusters[i].d, 66, 1);

            // fclose(fpDebug);

            // int debug = 0;
        }
    }

    if (bMergeClusters)
    {

        // Calculating distance table - '1' if proximity criterion holds for CTIs d_i and d_j
        int *pCTIDistanceTable = new int[nClusters * nClusters];
        float CTIDistance, minCTIDistance;

        float *d1, *d2;

        // DEBUX TXT files
        FILE *fpTable;
        if (verbose)
            fpTable = fopen("DEBUGtable.txt", "w+");

        for (int i = 0; i < nClusters; i++)
        {
            d1 = MergedSClusters[i].d;

            for (int j = 0; j < nClusters; j++)
            {
                // same cluster types (CONCEX - CONCAVE) and both are valid
                if ((MergedSClusters[i].flags & (RVLPCSEGMENT_OBJECT_FLAG_CONCAVE | RVLPCSEGMENT_OBJECT_FLAG_VALID)) == (MergedSClusters[j].flags & (RVLPCSEGMENT_OBJECT_FLAG_CONCAVE | RVLPCSEGMENT_OBJECT_FLAG_VALID)))
                {
                    // d(i,j) == d(j,i)
                    if (i < j)
                    {
                        if (MergedSClusters[i].label == MergedSClusters[j].label)
                        {
                            d2 = MergedSClusters[j].d;

                            minCTIDistance = 1.1; // change value

                            for (int k = 0; k < alignment.convexTemplate.n; k++)
                            {
                                iCTIPair = (k + 33) % alignment.convexTemplate.n;
                                CTIDistance = (d1[k] + d2[iCTIPair]) / (RVLMIN(d1[k] + d1[iCTIPair], d2[k] + d2[iCTIPair]));

                                if (CTIDistance < minCTIDistance)
                                {
                                    minCTIDistance = CTIDistance;
                                }
                            }

                            if (-proximityThresh > minCTIDistance)
                            {
                                // d(i, j) == d(j,i)
                                pCTIDistanceTable[i * nClusters + j] = 0;
                                pCTIDistanceTable[j * nClusters + i] = 0;

                                if (verbose)
                                {
                                    fprintf(fpTable, "(%d,%d) -> %d\t", i, j, 0);
                                    fprintf(fpTable, "(%d,%d) -> %d\t", j, i, 0);
                                    fprintf(fpTable, "%f\n", minCTIDistance);
                                }
                            }
                            // clusters are candidates for merging if minCTIDistance >= -proximityThresh
                            else
                            {
                                // d(i, j) == d(j,i)
                                pCTIDistanceTable[i * nClusters + j] = 1;
                                pCTIDistanceTable[j * nClusters + i] = 1;

                                if (verbose)
                                {
                                    fprintf(fpTable, "(%d,%d) -> %d\t", i, j, 1);
                                    fprintf(fpTable, "(%d,%d) -> %d\t", j, i, 1);
                                    fprintf(fpTable, "%f\n", minCTIDistance);
                                }
                            }
                        }
                        // if part labels are not same
                        else
                        {
                            pCTIDistanceTable[i * nClusters + j] = 0;
                            pCTIDistanceTable[j * nClusters + i] = 0;

                            if (verbose)
                            {
                                fprintf(fpTable, "(%d,%d) -> %d\n", i, j, 0);
                                fprintf(fpTable, "(%d,%d) -> %d\n", j, i, 0);
                            }
                        }
                    }
                    // diagonal
                    else if (i == j)
                    {
                        pCTIDistanceTable[i * nClusters + j] = 0; // cluster can not be merged with himself

                        if (verbose)
                            fprintf(fpTable, "(%d,%d) -> %d\n", i, j, 0);
                    }
                }
                // different cluster types (CONCEX - CONCAVE)
                else
                {
                    pCTIDistanceTable[i * nClusters + j] = 0;

                    if (verbose)
                        fprintf(fpTable, "(%d,%d) -> Different cluster types\n", i, j);
                }
            }
        }

        if (verbose)
            fclose(fpTable);

        // Calculate Bounding Sphere
        float boundingSphereCenter[3];
        float boundingSphereRadius;

        // Copy vertices from pSurfels->vertexArray to convexHullVertices - for complete mesh
        float *P, *P_, *Pmax;
        float Pdist[3];
        float *N;

        for (int i = 0; i < pSurfels->vertexArray.n; i++)
        {
            P = pSurfels->vertexArray.Element[i]->P;

            P_ = convexHullVertices.Element + 3 * i;

            RVLCOPY3VECTOR(P, P_);
        }

        convexHullVertices.h = pSurfels->vertexArray.n;

        BoundingSphere<float>(convexHullVertices, boundingSphereCenter, boundingSphereRadius);

        int *pStartIdx = new int[nClusters];
        pStartIdx[0] = 0;
        int endIdx, idx;
        Mesh mergedClustersConvexHull;
        MESH::Face *pFace;
        bool bContinueMerging = false;
        bool bFirstMerge = true;
        bool bMerge = false;
        bool bMerge2 = false;
        bool bRemoveBaseClusters = false;
        RVL::QLIST::Index2 *pt;
        // float Pt[3];
        int ptCnt = 0;
        float e;
        int outlierCnt;
        float outlierPercent;
        int nPts;

        // allocate array for storing distance from points to the convex hull
        Array<float> emax;
        emax.Element = new float[pMesh->NodeArray.n];
        emax.n = 0;

        float e_;

        if (verbose)
            printf("\n");

        // Merge clusters
        for (int n = 0; n < nClusters; n++)
        {
            endIdx = MergedSClusters.size();
            bFirstMerge = true;
            bContinueMerging = false;

            for (int i = pStartIdx[n]; i < endIdx; i++)
            {
                if (!CheckFlag(MergedSClusters[i].flags, RVLPCSEGMENT_OBJECT_FLAG_VALID))
                    continue;

                for (int j = (MergedSClusters[i].iClusterArray.Element[MergedSClusters[i].iClusterArray.n - 1] + 1); j < nClusters; j++)
                {
                    if (!CheckFlag(MergedSClusters[i].flags, RVLPCSEGMENT_OBJECT_FLAG_VALID))
                        continue;

                    if (!CheckFlag(MergedSClusters[j].flags, RVLPCSEGMENT_OBJECT_FLAG_VALID))
                        continue;

                    bMerge = false;

                    // checking proximity criterion for clusters i and j
                    if (verbose)
                        printf("Checking distance criterion for clusters %d and %d: ", i, j);

                    for (int iCluster = 0; iCluster < MergedSClusters[i].iClusterArray.n; iCluster++)
                    {
                        idx = MergedSClusters[i].iClusterArray.Element[iCluster];

                        if (verbose)
                            if (idx != i)
                                printf("\n\tChecking distance criterion for clusters %d and %d: ", idx, j);

                        if (pCTIDistanceTable[idx * nClusters + j])
                        {
                            bMerge = true;
                            break;
                        }
                    }

                    if (bMerge)
                    {
                        if (verbose)
                            printf("YES!\n");

                        float o = (MergedSClusters[i].flags & RVLPCSEGMENT_OBJECT_FLAG_CONCAVE ? -1.0f : 1.0f);

                        // create union of veretices
                        RVL::UnionOfIndices(MergedSClusters[i].iVertexArray, MergedSClusters[j].iVertexArray, iVertexArray, bVertexInArray);

                        // create union of surfels
                        RVL::UnionOfIndices(MergedSClusters[i].iSurfelArray, MergedSClusters[j].iSurfelArray, iSurfelArray, bSurfelInArray);

                        // Copy vertices from pSurfels->vertexArray to convexHullVertices.
                        for (int iVertex = 0; iVertex < iVertexArray.n; iVertex++)
                        {
                            P = pSurfels->vertexArray.Element[iVertexArray.Element[iVertex]]->P;

                            P_ = convexHullVertices.Element + 3 * iVertex;

                            RVLCOPY3VECTOR(P, P_);
                        }

                        convexHullVertices.h = iVertexArray.n;

                        // calculate convex hull
                        mergedClustersConvexHull.ConvexHull(convexHullVertices, &memConvexHull);

                        // if (i == 2 && j == 5)
                        //{
                        //	Visualizer vis;

                        //	vis.Create();

                        //	mergedClustersConvexHull.CreateVTKPolyData();

                        //	vis.AddMesh(mergedClustersConvexHull.pPolygonData);

                        //	vis.Run();
                        //}

                        // Point counter for merged cluster
                        ptCnt = 0;

                        // calculate distance from convex hull
                        for (int iSurfel = 0; iSurfel < iSurfelArray.n; iSurfel++)
                        {
                            idx = iSurfelArray.Element[iSurfel];
                            pt = pSurfels->NodeArray.Element[idx].PtList.pFirst;

                            // for all points in surfel
                            while (pt)
                            {
                                P = pMesh->NodeArray.Element[pt->Idx].P;

                                if (bDistanceToCHUsingNormals)
                                {
                                    N = pMesh->NodeArray.Element[pt->Idx].N;
                                    emax.n = ptCnt + 1;

                                    // first valid vertex
                                    P_ = mergedClustersConvexHull.NodeArray.Element[mergedClustersConvexHull.iValidVertices.Element[0]].P;
                                    e_ = RVLDOTPRODUCT3(N, P_);
                                    Pmax = P_;

                                    int nValidVertices = mergedClustersConvexHull.iValidVertices.n;
                                    int iValidVertex;

                                    // for all other CH vertices find vertex with maximum value of nT*P_
                                    for (int iVertex = 1; iVertex < nValidVertices; iVertex++)
                                    {
                                        iValidVertex = mergedClustersConvexHull.iValidVertices.Element[iVertex];
                                        P_ = mergedClustersConvexHull.NodeArray.Element[iValidVertex].P;

                                        e = RVLDOTPRODUCT3(N, P_);

                                        if (o * e > o * e_)
                                        {
                                            e_ = e;
                                            Pmax = P_;
                                        }
                                    }

                                    // calculate distance for point P and vertex Pmax
                                    RVLDIF3VECTORS(P, Pmax, Pdist);
                                    emax.Element[ptCnt] = RVLABS(RVLDOTPRODUCT3(N, Pdist));
                                }
                                // bDistanceToCHUsingNormals = false
                                else
                                {

                                    // for the first face of convex hull
                                    pFace = mergedClustersConvexHull.faces.Element[0];
                                    emax.Element[ptCnt] = (pFace->N[0] * P[0] + pFace->N[1] * P[1] + pFace->N[2] * P[2]) - pFace->d;
                                    emax.n = ptCnt + 1;

                                    // for all convex hull faces
                                    for (int iFace = 1; iFace < mergedClustersConvexHull.faces.n; iFace++)
                                    {
                                        pFace = mergedClustersConvexHull.faces.Element[iFace];

                                        e = (pFace->N[0] * P[0] + pFace->N[1] * P[1] + pFace->N[2] * P[2]) - pFace->d;

                                        if (e > emax.Element[ptCnt])
                                            emax.Element[ptCnt] = e;
                                    }

                                    emax.Element[ptCnt] = RVLABS(emax.Element[ptCnt]);
                                }

                                ptCnt++;
                                pt = pt->pNext;
                            } // for all points in surfel
                        }     // for all surfels

                        outlierCnt = 0;

                        // check convex hull criterion
                        for (int iE = 0; iE < emax.n; iE++)
                        {
                            if (emax.Element[iE] > (CHDistanceThresh * boundingSphereRadius))
                                outlierCnt++;
                        }

                        // Check merge criterion
                        nPts = MergedSClusters[i].size > MergedSClusters[j].size ? MergedSClusters[j].size : MergedSClusters[i].size;
                        // outlierPercent = (float)outlierCnt / (float)emax.n;
                        outlierPercent = (float)outlierCnt / (float)nPts;

                        if (outlierPercent < mergeThresh1)
                        {
                            if (verbose)
                                printf("Remove base cluster and create complex cluster!\n");

                            bMerge2 = true;
                            bRemoveBaseClusters = true;
                        }
                        else if ((outlierPercent >= mergeThresh1) && (outlierPercent < mergeThresh2))
                        {
                            if (verbose)
                                printf("Create complex cluster but keep base cluster!\n");

                            bMerge2 = true;
                            bRemoveBaseClusters = false;
                        }
                        else
                        {
                            if (verbose)
                                printf("Convex hull criterion is not met for clusters %d and %d (outlier: %f)\n", i, j, outlierPercent);

                            bMerge2 = false;
                        }

                        if (bMerge2)
                        {
                            // pObjectCluster = new SURFEL::Object;

                            // add new cluster to array of merged clusters
                            // MergedSClusters.push_back(*pObjectCluster);
                            MergedSClusters.push_back(Object);

                            // allocate memory for storing cluster indices of the new cluster
                            MergedSClusters[MergedSClusters.size() - 1].iClusterArray.Element = new int[nClusters];
                            // copy cluster indices from the complex cluster to the new cluster
                            memcpy(MergedSClusters[MergedSClusters.size() - 1].iClusterArray.Element, MergedSClusters[i].iClusterArray.Element, sizeof(int) * MergedSClusters[i].iClusterArray.n);
                            // add j-th cluster index to the new cluster
                            MergedSClusters[MergedSClusters.size() - 1].iClusterArray.Element[MergedSClusters[i].iClusterArray.n] = MergedSClusters[j].iClusterArray.Element[0];
                            // set number of clusters in the new cluster
                            MergedSClusters[MergedSClusters.size() - 1].iClusterArray.n = MergedSClusters[i].iClusterArray.n + 1;

                            // copy vertex array to the new cluster
                            MergedSClusters[MergedSClusters.size() - 1].iVertexArray.Element = new int[iVertexArray.n];
                            memcpy(MergedSClusters[MergedSClusters.size() - 1].iVertexArray.Element, iVertexArray.Element, sizeof(int) * iVertexArray.n);
                            MergedSClusters[MergedSClusters.size() - 1].iVertexArray.n = iVertexArray.n;

                            // copy surfel array to the new cluster
                            MergedSClusters[MergedSClusters.size() - 1].iSurfelArray.Element = new int[iSurfelArray.n];
                            memcpy(MergedSClusters[MergedSClusters.size() - 1].iSurfelArray.Element, iSurfelArray.Element, sizeof(int) * iSurfelArray.n);
                            MergedSClusters[MergedSClusters.size() - 1].iSurfelArray.n = iSurfelArray.n;

                            // set flags for the new cluster
                            MergedSClusters[MergedSClusters.size() - 1].flags = MergedSClusters[i].flags;

                            // merge descriptors
                            MergedSClusters[MergedSClusters.size() - 1].d = new float[alignment.convexTemplate.n];

                            // if (MergedSClusters.size() - 1 == 23)
                            //	int debug = 0;

                            if (CheckFlag(MergedSClusters[MergedSClusters.size() - 1].flags, RVLPCSEGMENT_OBJECT_FLAG_CONCAVE))
                                MergeDescriptors(MergedSClusters[i].d, MergedSClusters[j].d, -1.0, MergedSClusters[MergedSClusters.size() - 1].d);
                            else
                                MergeDescriptors(MergedSClusters[i].d, MergedSClusters[j].d, 1.0, MergedSClusters[MergedSClusters.size() - 1].d);

                            // if (MergedSClusters.size() - 1 == 23)
                            //{
                            //	FILE *fpDebug = fopen("C:\\RVL\\ExpRez\\d.txt", "w");

                            //	PrintMatrix<float>(fpDebug, MergedSClusters[MergedSClusters.size() - 1].d, 66, 1);

                            //	fclose(fpDebug);

                            //	int debug = 0;
                            //}

                            // merge bd-s
                            MergedSClusters[MergedSClusters.size() - 1].bd = new uchar[alignment.convexTemplate.n];

                            memset(MergedSClusters[MergedSClusters.size() - 1].bd, 0x01, sizeof(uchar) * alignment.convexTemplate.n);

                            // set merged cluster size
                            MergedSClusters[MergedSClusters.size() - 1].size = ptCnt;

                            // set merged cluster label
                            MergedSClusters[MergedSClusters.size() - 1].label = MergedSClusters[i].label;

                            BYTE debugFlags = MergedSClusters[i].flags;
                            debugFlags = MergedSClusters[MergedSClusters.size() - 1].flags;

                            // debug
                            if (verbose)
                                printf("New cluster created: %d = %d & %d (outlier: %f)\n", MergedSClusters.size() - 1, i, j, outlierPercent);

                            // remove VALID flag from the base clusters
                            if (bRemoveBaseClusters)
                            {
                                MergedSClusters[i].flags = MergedSClusters[i].flags & (~RVLPCSEGMENT_OBJECT_FLAG_VALID);
                                MergedSClusters[j].flags = MergedSClusters[j].flags & (~RVLPCSEGMENT_OBJECT_FLAG_VALID);
                                if (verbose)
                                    printf("VALID flag removed from clusters: %d and %d\n", i, j);
                            }

                            debugFlags = MergedSClusters[MergedSClusters.size() - 1].flags;

                            // if this is first cluster with n+2 clusters
                            if (bFirstMerge)
                            {
                                pStartIdx[n + 1] = MergedSClusters.size() - 1;
                                bFirstMerge = false;
                                bContinueMerging = true;
                            }
                        }
                    } // if (bMerge)
                    else
                    {
                        if (verbose)
                            printf("NO!\n");
                    }
                }
            }

            if (!bContinueMerging)
            {
                if (verbose)
                    printf("No clusters were merged in the last iteration!\n");
                break;
            }
        }

        if (verbose)
            printf("Cluster merging finished!\n");

        delete[] pCTIDistanceTable;
        delete[] pStartIdx;
        delete[] emax.Element;
    }

    if (verbose)
        printf("List of merged clusters: ");

    int nValidClusters = 0;

    for (int iCluster = 0; iCluster < MergedSClusters.size(); iCluster++)
    {
        if (CheckFlag(MergedSClusters[iCluster].flags, RVLPCSEGMENT_OBJECT_FLAG_VALID))
        {
            nValidClusters++;

            if (verbose)
            {
                printf("\n%d) %d ", nValidClusters - 1, iCluster);

                if (MergedSClusters[iCluster].iClusterArray.n > 1)
                {
                    printf("<-> ");
                    for (int iMergedCluster = 0; iMergedCluster < MergedSClusters[iCluster].iClusterArray.n; iMergedCluster++)
                    {
                        printf("%d ", MergedSClusters[iCluster].iClusterArray.Element[iMergedCluster]);
                    }
                }
            }
        }
    }

    if (verbose)
        printf("\n");

    // Clear pObjects
    if (pObjects->objectArray.Element)
    {
        for (int iObject = 0; iObject < pObjects->objectArray.n; iObject++)
        {
            delete[] pObjects->objectArray.Element[iObject].iClusterArray.Element;
            delete[] pObjects->objectArray.Element[iObject].iSurfelArray.Element;
            delete[] pObjects->objectArray.Element[iObject].iVertexArray.Element;
            delete[] pObjects->objectArray.Element[iObject].d;
            delete[] pObjects->objectArray.Element[iObject].bd;

            if (verbose && iObject == pObjects->objectArray.n - 1)
                printf("pObjects MEM cleared!\n");
        }

        delete[] pObjects->objectArray.Element;
    }

    // Copy VALID objects from MergedSClusters to pObjects

    pObjects->objectArray.Element = new SURFEL::Object[nValidClusters];
    SURFEL::Object *pObject = &pObjects->objectArray.Element[0];
    pObjects->objectArray.n = 0;
    RVL_DELETE_ARRAY(pObjects->iObjectAssignedToNode);
    pObjects->iObjectAssignedToNode = new int[nValidClusters];
    int totalnSurfels = 0;

    for (int iCluster = 0; iCluster < MergedSClusters.size(); iCluster++)
    {
        if (CheckFlag(MergedSClusters[iCluster].flags, RVLPCSEGMENT_OBJECT_FLAG_VALID))
        {
            *pObject = MergedSClusters[iCluster];

            pObjects->iObjectAssignedToNode[pObjects->objectArray.n] = pObjects->objectArray.n;

            pObjects->objectArray.n++;

            totalnSurfels += pObject->iSurfelArray.n;

            pObject++;
        }
    }

    QLIST::Index *iSurfelMem;

    RVLMEM_ALLOC_STRUCT_ARRAY(pMem, QLIST::Index, totalnSurfels, iSurfelMem);

    QLIST::Index *piSurfel = iSurfelMem;

    int i, iObject;
    QList<QLIST::Index> *piSurfelList;

    for (iObject = 0; iObject < pObjects->objectArray.n; iObject++)
    {
        pObject = pObjects->objectArray.Element + iObject;

        piSurfelList = &(pObject->surfelList);

        RVLQLIST_INIT(piSurfelList);

        for (i = 0; i < pObject->iSurfelArray.n; i++)
        {
            RVLQLIST_ADD_ENTRY(piSurfelList, piSurfel);

            piSurfel->Idx = pObject->iSurfelArray.Element[i];

            piSurfel++;
        }
    }

    if (!verbose)
        printf("completed\n");

    printf("No. of valid clusters = %d\n", nValidClusters);

    // delete[] pCTIDistanceTable;
    // delete[] pStartIdx;
    // delete[] emax.Element;
}

// Calculates color histogram for every segment
// void VNClassifier::UpdateSegmentsColorHistograms()
//{
//	SURFEL::Object *pSegment;
//	Surfel *pCurrSurfel;
//
//	for (int iSegment = 0; iSegment < pObjects->objectArray.n; iSegment++)	//For all segments
//	{
//		pSegment = &pObjects->objectArray.Element[iSegment];
//
//		pSegment->colordescriptor = std::make_shared<RVLColorDescriptor>(RVLColorDescriptor());
//
//		if (this->pSurfels->NodeArray.Element[pSegment->iSurfelArray.Element[0]].colordescriptor)
//			*pSegment->colordescriptor = *this->pSurfels->NodeArray.Element[pSegment->iSurfelArray.Element[0]].colordescriptor;	//Set starting descriptor from first surfel
//
//		if (pSegment->iSurfelArray.n < 2)	//If there is only one surfel in cluster then finish with this cluster
//			continue;
//
//		for (int iSurfel = 1; iSurfel < pSegment->iSurfelArray.n; iSurfel++)	//Add all other descriptors
//		{
//			pCurrSurfel = &this->pSurfels->NodeArray.Element[pSegment->iSurfelArray.Element[iSurfel]];
//			if ((pCurrSurfel->size <= 1) || pCurrSurfel->bEdge)
//				continue;
//
//			*pSegment->colordescriptor += *pCurrSurfel->colordescriptor;
//		}
//
//		pSegment->colordescriptor->DisplayColorHistogram(true);
//	}
// }

// Matches scene color descriptor to iModel color descriptor using corresponding metric
float VNClassifier::CHMatching(Array<int> iSSurfelArray, int iModel, int metric, FILE *fp)
{
    Surfel *pSurfel;
    RVLColorDescriptor colorDescriptor;
    float CHMetric;
    bool bFirstDescriptor = true;

    // set color descriptor to the color descriptor of the first surfel in iSSurfelArray
    if (iSSurfelArray.n > 0)
    {
        // if(pSurfels->NodeArray.Element[iSSurfelArray.Element[0]].colordescriptor)
        //	colorDescriptor = *pSurfels->NodeArray.Element[iSSurfelArray.Element[0]].colordescriptor;

        // running through iSSurfelArray and create color descriptor for all surfels in iSSurfelArray
        for (int iSurfel = 0; iSurfel < iSSurfelArray.n; iSurfel++)
        {
            pSurfel = &pSurfels->NodeArray.Element[iSSurfelArray.Element[iSurfel]];

            // Update CH with CH of the current surfel
            if (pSurfel->colordescriptor)
                if (bFirstDescriptor)
                {
                    colorDescriptor = *pSurfel->colordescriptor;

                    bFirstDescriptor = false;
                }
                else
                    colorDescriptor += *pSurfel->colordescriptor;
        }

        // save descriptor to file
        if (fp)
            colorDescriptor.SaveCH2File(fp);

        // Test CH Vizualization
        // colorDescriptor.TestCHVisualization(RVLColorDescriptor::ColorSpaceList::HSV);

        // Display scene color descriptor
        // colorDescriptor.DisplayColorHistogram(true);

        // Display model color descriptor
        // pShapeInstanceDetection->cdModelDB.at(iModel)->DisplayColorHistogram(true);

        // calculate Metric
        CHMetric = pShapeInstanceDetection->cdModelDB.at(iModel)->CalculateCHMetric(colorDescriptor, metric);

        if (CHMetric != -1.0f)
            return CHMetric;
    }

    if (metric == RVLColorDescriptor::MetricsList::Bhattacharyya)
        return 10.0;
    else
        return 0;
}
// END Vidovic

void VNClassifier::CreatePartModel()
{
    QList<VNInstance> *pModelVNList = &modelVNList;

    VNInstance *pVNModelInstance;
    pVNModelInstance = pModelVNList->pFirst;

    int maxnComponents = 0;
    int nModels = 0;
    int nMComponents = 0;
    int maxLabel = 0;
    while (pVNModelInstance)
    {
        nMComponents += pVNModelInstance->nComponents;
        nModels++;

        for (int iC = 0; iC < pVNModelInstance->nComponents; iC++)
        {
            if (pVNModelInstance->label[iC] > maxLabel)
                maxLabel = pVNModelInstance->label[iC];
        }

        if (pVNModelInstance->nComponents > maxnComponents)
            maxnComponents = pVNModelInstance->nComponents;

        pVNModelInstance = pVNModelInstance->pNext;
    }

    int nLabels = maxLabel + 1;

    classes[0].nLabels = nLabels;

    int m = RVLMAX(classArray.Element[0].M.h, classArray.Element[1].M.h);

    int ms = m - 3;

    // Match all models with each other.

    printf("Model matching...");

    Array<int> *correspMatrix = new Array<int>[nModels];
    int *correspMem = new int[nModels * nMComponents];
    int *pCorrespMatrix = correspMem;
    int iModel1 = 0;
    pVNModelInstance = pModelVNList->pFirst;

    float *E, *e;
    VN_::ModelTemplate2 ModelTemplateTemplate;
    E = new float[nModels * nModels * nLabels];

    while (pVNModelInstance)
    {
        correspMatrix[iModel1].Element = pCorrespMatrix;

        int *Correspondences;

        if (iModel1 > 0)
        {
            pVNModelInstance->Match(pModelVNList, Correspondences, this, E, correspMatrix, iModel1, nModels);
            delete[] Correspondences;
        }

        pCorrespMatrix += (nModels * pVNModelInstance->nComponents);

        iModel1++;
        pVNModelInstance = pVNModelInstance->pNext;
    }

    printf("completed.");

    // Create metamodels.

    int maxno = 2;
    int maxnu = 2;

    classes[0].metamodels = new std::vector<VN_::ModelTemplate2>[nLabels];

    QLIST::Entry<Pair<int, int>> *modelTemplateCorrespListMem = new QLIST::Entry<Pair<int, int>>[nModels * nModels * nLabels];
    QLIST::Entry<Pair<int, int>> *pModelTemplateCorresp = modelTemplateCorrespListMem;

    int i, iLabel, iModel2, iMM;
    int no, nu;
    std::vector<RECOG::VN_::ModelTemplate2> *modelTemplateVector = classes[0].metamodels;
    QList<QLIST::Entry<Pair<int, int>>> *pModelTemplateCorrespList;
    VN_::ModelTemplate2 modelTemplate;

    for (iLabel = 0; iLabel <= maxLabel; iLabel++)
    {
        pVNModelInstance = pModelVNList->pFirst;
        for (iModel1 = 0; iModel1 < nModels; iModel1++)
        {
            for (iModel2 = iModel1 + 1; iModel2 < nModels; iModel2++)
            {
                no = 0;
                nu = 0;
                for (i = 0; i < pVNModelInstance->nComponents; i++)
                {
                    if (pVNModelInstance->label[i] == iLabel)
                    {
                        if (RVLVN_GET_CORRESPONDENCE(correspMatrix, modelVNArray, iModel1, iModel2, i) != -1)
                        {
                            if (pVNModelInstance->bConcave[i] > 0)
                                nu++;
                            else
                                no++;
                        }
                    }
                }

                bool bExists = false;
                for (iMM = 0; iMM < modelTemplateVector[iLabel].size(); iMM++)
                {
                    if (modelTemplateVector[iLabel][iMM].label == iLabel)
                    {
                        if (modelTemplateVector[iLabel][iMM].no == no && modelTemplateVector[iLabel][iMM].nu == nu)
                        {
                            modelTemplateVector[iLabel][iMM].f++;
                            bExists = true;
                            pModelTemplateCorrespList = &(modelTemplateVector[iLabel][iMM].correspondences);
                            break;
                        }
                    }
                }
                if (!bExists)
                {
                    ModelTemplateTemplate.f = 1;
                    ModelTemplateTemplate.no = no;
                    ModelTemplateTemplate.nu = nu;
                    ModelTemplateTemplate.label = iLabel;
                    ModelTemplateTemplate.conditionalProbability.Element = NULL;
                    modelTemplateVector[iLabel].push_back(ModelTemplateTemplate);
                    pModelTemplateCorrespList = &(modelTemplateVector[iLabel][modelTemplateVector[iLabel].size() - 1].correspondences);
                    RVLQLIST_INIT(pModelTemplateCorrespList);
                }

                pModelTemplateCorresp->data.a = iModel1;
                pModelTemplateCorresp->data.b = iModel2;
                RVLQLIST_ADD_ENTRY(pModelTemplateCorrespList, pModelTemplateCorresp);
                pModelTemplateCorresp++;
            }
            pVNModelInstance = pVNModelInstance->pNext;
        }

        for (iMM = 0; iMM < modelTemplateVector[iLabel].size(); iMM++)
        {
            modelTemplate = modelTemplateVector[iLabel][iMM];

            if (modelTemplate.no == 0 && modelTemplate.nu == 0 || modelTemplate.no > maxno || modelTemplate.nu > maxnu || modelTemplate.f < 5)
                modelTemplateVector[iLabel].erase(modelTemplateVector[iLabel].begin() + (iMM--));
        }
    }

    ///// Create model clusters.

    // Parameters.

    int maxnModelClusters = 1000;

    //

    printf("Creating model clusters:\n\n");

    int nOrigMMsTotal = 0;

    for (iLabel = 0; iLabel <= maxLabel; iLabel++)
        nOrigMMsTotal += modelTemplateVector[iLabel].size();

    Array2D<VN_::Gauss> *gauss = new Array2D<VN_::Gauss>[nOrigMMsTotal];
    float **gaussMem = new float *[nOrigMMsTotal];

    int maxnModelClustersExp = maxnModelClusters + 1;

    Array<VNInstance *> modelClusters;
    modelClusters.Element = new VNInstance *[maxnModelClustersExp];
    VNInstance *modelClustersMem = new VNInstance[maxnModelClustersExp];

    int iCluster;

    for (iCluster = 0; iCluster < maxnModelClustersExp; iCluster++)
        modelClusters.Element[iCluster] = modelClustersMem + iCluster;

    Array<int> *modelClusterPartMem = new Array<int>[2 * maxnModelClustersExp];

    Array<QList<VN_::ModelTemplateInstance>> ModelTemplateInstanceListArr;
    ModelTemplateInstanceListArr.Element = new QList<VN_::ModelTemplateInstance>[nModels];
    QList<VN_::ModelTemplateInstance> *pModelTemplateInstanceList;

    Array<int> components1[2];
    components1[0].Element = new int[maxnComponents];
    components1[1].Element = new int[maxnComponents];

    float minEC = 0.0f;

#ifdef RVLVN_PART_SEGMENTATION_TRAINING_LOG
    char clusteringLogFileName[] = "LMM.cls";
    char MSTLogFileName[] = "LMM.mst";

    FILE *fpLMM = fopen((std::string(resultsFolder) + "\\LMM.dat").data(), "w");
#endif

    int nGaussTotal = 0;
    int nMMGaussTotal = 0;

    int iOrigMMAbs = 0;
    int iMMAbs = 0;

    std::vector<Array<int>> metaModelGauss;

    Array<int> metaModelGaussBuff;
    metaModelGaussBuff.Element = new int[maxnModelClusters];

    Array<int> metaModelGaussArrayTemplate;

    Array<GRAPH::MSTreeEdge> MSTEdges;
    MSTEdges.Element = new GRAPH::MSTreeEdge[maxnModelClusters];

    int iComp1, iComp2, iCluster1, iCluster2, iCompC;
    int nModelClusters, nComponentsC, iModelCluster1, iModelCluster2, gaussBlockSize;
    VNInstance modelCluster;
    float EC_, clusterSize1, clusterSize2, newClusterSize, s2;
    int *modelClusterPartComponentIdxMem, *pModelClusterPartComponents, *CorrespondenceC;
    Pair<int, int> closestClusters;
    // float *d1, *d2;
    float *q1, *q2, *qs1;
    VNInstance *pModelCluster1, *pModelCluster2;
    std::vector<VN_::ModelTemplate2>::iterator pMetaModel;

    for (iLabel = 0; iLabel <= maxLabel; iLabel++)
    // for (int iLabel = 0; iLabel <= 2; iLabel++)
    {
        printf("Label %d\n", iLabel);

        int nOriginalMMs = modelTemplateVector[iLabel].size();

        int a, b;
        QLIST::Entry<Pair<int, int>> *Mcorrespondences;

        for (pMetaModel = modelTemplateVector[iLabel].begin(); pMetaModel != modelTemplateVector[iLabel].end(); ++pMetaModel)
        {
            printf("Metamodel %d//%d\n", iMM, modelTemplateVector[iLabel].size());

            modelTemplate = *pMetaModel;

            no = modelTemplate.no;
            nu = modelTemplate.nu;

#ifdef RVLVN_PART_SEGMENTATION_TRAINING_LOG
            fprintf(fpLMM, "%d\t%d\t%d\n", iLabel, no, nu);
#endif

            // ModelTemplateInstanceListArr <- all variants of all models compatible with the model template iMM of the part iLabel.

            RVLMEM_ALLOC_LOCAL_INIT(pMem);

            for (i = 0; i < nModels; i++)
            {
                // QList for each model
                pModelTemplateInstanceList = ModelTemplateInstanceListArr.Element + i;
                RVLQLIST_INIT(pModelTemplateInstanceList);
            }

            Mcorrespondences = modelTemplate.correspondences.pFirst;
            int iEdge = 0;
            int j, ou;
            Array<int> components1_, components2;
            VN_::ModelTemplateInstance *pModelTemplateInstance;
            int nModelTemplateInstance = 0;
            while (Mcorrespondences)
            {
                iModel1 = Mcorrespondences->data.a;
                iModel2 = Mcorrespondences->data.b;

                pVNModelInstance = modelVNArray.Element[iModel1];

                for (ou = 0; ou < 2; ou++)
                {
                    components1[ou].n = 0;

                    components1_ = pVNModelInstance->part.Element[ou * pVNModelInstance->part.w + iLabel];

                    for (i = 0; i < components1_.n; i++)
                    {
                        iComp1 = components1_.Element[i];
                        iComp2 = RVLVN_GET_CORRESPONDENCE(correspMatrix, modelVNArray, iModel1, iModel2, iComp1);
                        if (iComp2 == -1)
                            continue;

                        components1[ou].Element[components1[ou].n++] = iComp1;
                    }
                }

                pModelTemplateInstanceList = ModelTemplateInstanceListArr.Element + iModel1;

                pModelTemplateInstance = pModelTemplateInstanceList->pFirst;

                while (pModelTemplateInstance)
                {
                    for (ou = 0; ou < 2; ou++)
                    {
                        components2 = pModelTemplateInstance->iComponent[ou];

                        for (i = 0; i < components1[ou].n; i++)
                        {
                            iComp1 = components1[ou].Element[i];

                            for (j = 0; j < components2.n; j++)
                            {
                                if (iComp1 == components2.Element[j])
                                    break; // Component of components1[ou] is contained in components2.
                            }

                            if (j >= components2.n)
                                break; // Component iComp1 of components1[ou] is NOT contained in components2.
                        }

                        if (i < components1[ou].n)
                            break; // There is a component of components1[ou] which is NOT contained in components2.
                    }

                    if (ou >= 2)
                        break; // All components of components1 are contained in pModelTemplateInstance->iComponent,
                               // i.e. pModelTemplateInstance is equal to the currently examined variant.

                    pModelTemplateInstance = pModelTemplateInstance->pNext;
                }

                if (pModelTemplateInstance == NULL)
                {
                    // If new.

                    RVLMEM_ALLOC_STRUCT(pMem, VN_::ModelTemplateInstance, pModelTemplateInstance);
                    pModelTemplateInstance->iModel2 = iModel2;
                    pModelTemplateInstance->iComponent[0].n = no;
                    pModelTemplateInstance->iComponent[1].n = nu;
                    RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, no, pModelTemplateInstance->iComponent[0].Element);
                    RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, nu, pModelTemplateInstance->iComponent[1].Element);

                    for (ou = 0; ou < 2; ou++)
                        for (i = 0; i < components1[ou].n; i++)
                            pModelTemplateInstance->iComponent[ou].Element[i] = components1[ou].Element[i];

                    RVLQLIST_ADD_ENTRY(pModelTemplateInstanceList, pModelTemplateInstance);

                    nModelTemplateInstance++;
                }

                Mcorrespondences = Mcorrespondences->pNext;
            } // for every correspondence in modelTemplateVector[iLabel][iMM]

            //// Create model Clusters of the model template iMM of the part iLabel.

#ifdef RVLVN_PART_SEGMENTATION_TRAINING_LOG
            clusteringLogFileName[0] = '0' + iLabel;
            clusteringLogFileName[1] = '0' + no;
            clusteringLogFileName[2] = '0' + nu;

            FILE *fpLog = fopen((std::string(resultsFolder) + "\\" + clusteringLogFileName).data(), "w");
#endif

            nModelClusters = RVLMIN(maxnModelClustersExp, nModelTemplateInstance);

            nComponentsC = no + nu;

            // float *dMem = new float[nModelClusters * nComponentsC * sampledUnitSphere.h];
            // bool *bdMem = new bool[nModelClusters * nComponentsC * sampledUnitSphere.h];
            float *qMem = new float[nModelClusters * nComponentsC * m];
            float *sMem = new float[nModelClusters * nComponentsC];
            bool *bConcaveMem = new bool[nModelClusters * nComponentsC];
            modelClusterPartComponentIdxMem = new int[nModelClusters * nComponentsC];

            float *EC = new float[nModelClusters * nModelClusters];

            Array<int> *correspMxC = new Array<int>[nModelClusters];
            int *correspMxCMem = new int[nModelClusters * nModelClusters * nComponentsC];

            int iModelCluster;

            for (iModelCluster = 0; iModelCluster < nModelClusters; iModelCluster++)
                correspMxC[iModelCluster].Element = correspMxCMem + iModelCluster * nModelClusters * nComponentsC;

            int *clusterSize = new int[nModelClusters];

            iModelCluster = 0;
            modelClusters.n = 0;
            int iComp;

            for (iModel1 = 0; iModel1 < nModels; iModel1++)
            {
                pModelTemplateInstance = (ModelTemplateInstanceListArr.Element + iModel1)->pFirst;

                while (pModelTemplateInstance)
                {
                    pVNModelInstance = modelVNArray.Element[iModel1];

                    // Create a new model cluster.

                    modelCluster.nComponents = nComponentsC;
                    modelCluster.bLabels = false;
                    modelCluster.part.w = 1;
                    modelCluster.part.h = 2;
                    modelCluster.part.Element = modelClusterPartMem + 2 * iModelCluster;
                    modelCluster.part.Element[0].n = no;
                    modelCluster.part.Element[1].n = nu;
                    // modelCluster.d = dMem + nComponentsC * sampledUnitSphere.h * iModelCluster;
                    // modelCluster.bd = bdMem + nComponentsC * sampledUnitSphere.h * iModelCluster;
                    modelCluster.q = qMem + nComponentsC * iModelCluster * m;
                    modelCluster.s = sMem + nComponentsC * iModelCluster;
                    modelCluster.bConcave = bConcaveMem + nComponentsC * iModelCluster;

                    iCompC = 0;

                    pModelClusterPartComponents = modelClusterPartComponentIdxMem + nComponentsC * iModelCluster;

                    for (ou = 0; ou < 2; ou++)
                    {
                        modelCluster.part.Element[ou].Element = pModelClusterPartComponents;

                        pModelClusterPartComponents += modelCluster.part.Element[ou].n;

                        for (i = 0; i < pModelTemplateInstance->iComponent[ou].n; i++)
                        {
                            iComp = pModelTemplateInstance->iComponent[ou].Element[i];

                            // memcpy(modelCluster.d + i * sampledUnitSphere.h, pVNModelInstance->d + iComp*sampledUnitSphere.h, sampledUnitSphere.h * sizeof(float));
                            // if (pVNModelInstance->bd)
                            //	memcpy(modelCluster.bd + i * sampledUnitSphere.h, pVNModelInstance->bd + iComp*sampledUnitSphere.h, sampledUnitSphere.h * sizeof(bool));
                            memcpy(modelCluster.q + iCompC * m, pVNModelInstance->q + iComp * m, m * sizeof(float));
                            modelCluster.s[iCompC] = pVNModelInstance->s[iComp];
                            modelCluster.bConcave[iCompC] = (ou == 1);
                            modelCluster.part.Element[ou].Element[i] = iCompC;
                            iCompC++;
                        }
                    }

                    memcpy(modelClusters.Element[iModelCluster], &modelCluster, sizeof(VNInstance));

                    clusterSize[iModelCluster] = 1;

#ifdef RVLVN_PART_SEGMENTATION_TRAINING_LOG
                    fprintf(fpLog, "%d\t", iModel1);

                    for (ou = 0; ou < 2; ou++)
                    {
                        for (i = 0; i < pModelTemplateInstance->iComponent[ou].n; i++)
                            fprintf(fpLog, "%d\t", pModelTemplateInstance->iComponent[ou].Element[i]);
                    }

                    fprintf(fpLog, "%d\t", iModelCluster);
#endif

                    // Match the new model cluster with all previously created model clusters.

                    modelCluster.Match(modelClusters, this, CorrespondenceC, nModelClusters, EC, correspMxC, iModelCluster, true);

                    delete[] CorrespondenceC;

                    //

                    if (modelClusters.n < maxnModelClusters)
                    {
                        iModelCluster++;
                        modelClusters.n++;

#ifdef RVLVN_PART_SEGMENTATION_TRAINING_LOG
                        for (i = 0; i < nComponentsC + 2; i++)
                            fprintf(fpLog, "%d\t", -1);
#endif
                    }
                    else
                    {
                        modelClusters.n = maxnModelClustersExp;

                        /// Update model clusters.

                        // Identify the most similar clusters.

                        closestClusters.a = -1;

                        for (iCluster1 = 0; iCluster1 < maxnModelClustersExp; iCluster1++)
                            for (iCluster2 = 0; iCluster2 < iCluster1; iCluster2++)
                            {
                                EC_ = EC[iCluster1 * maxnModelClustersExp + iCluster2];

                                if (EC_ < minEC || closestClusters.a < 0)
                                {
                                    minEC = EC_;
                                    closestClusters.a = iCluster1;
                                    closestClusters.b = iCluster2;
                                }
                            }

                        // Merge cluster closestClusters.b with cluster closestClusters.b.

                        pModelCluster1 = modelClusters.Element[closestClusters.a];
                        pModelCluster2 = modelClusters.Element[closestClusters.b];

                        clusterSize1 = (float)clusterSize[closestClusters.a];
                        clusterSize2 = (float)clusterSize[closestClusters.b];
                        clusterSize[closestClusters.a] += clusterSize[closestClusters.b];
                        newClusterSize = (float)clusterSize[closestClusters.a];

#ifdef RVLVN_PART_SEGMENTATION_TRAINING_LOG
                        fprintf(fpLog, "%d\t%d\t", closestClusters.a, closestClusters.b);
#endif

                        for (iComp1 = 0; iComp1 < nComponentsC; iComp1++)
                        {
                            iComp2 = RVLVN_GET_CORRESPONDENCE(correspMxC, modelClusters, closestClusters.a, closestClusters.b, iComp1);

                            // if (iComp2 < 0 || iComp2 >= nComponentsC)
                            //	int debug = 0;

                            // d1 = pModelCluster1->d + sampledUnitSphere.h * iComp1;
                            // d2 = pModelCluster2->d + sampledUnitSphere.h * iComp2;

                            // for (i = 0; i < sampledUnitSphere.h; i++)
                            //	d1[i] = (clusterSize1 * d1[i] + clusterSize2 * d2[i]) / newClusterSize;

                            q1 = pModelCluster1->q + m * iComp1;
                            q2 = pModelCluster2->q + m * iComp2;

                            for (i = 0; i < m; i++)
                                q1[i] = (clusterSize1 * q1[i] + clusterSize2 * q2[i]) / newClusterSize;

                            qs1 = q1 + 3;

                            RVLDOTPRODUCT(qs1, qs1, ms, s2, i);

                            pModelCluster1->s[iComp1] = sqrt(s2);

#ifdef RVLVN_PART_SEGMENTATION_TRAINING_LOG
                            fprintf(fpLog, "%d\t", iComp2);
#endif
                        }

                        // Update match costs and correspondences.

                        pModelCluster1->Match(modelClusters, this, CorrespondenceC, modelClusters.n, EC, correspMxC, closestClusters.a, true);

                        delete[] CorrespondenceC;

                        // Set the index of the next new cluster.

                        iModelCluster = closestClusters.b;

                        ///
                    }

#ifdef RVLVN_PART_SEGMENTATION_TRAINING_LOG
                    fprintf(fpLog, "\n");
#endif

                    pModelTemplateInstance = pModelTemplateInstance->pNext;
                } // For all model template instances of iModel1
            }     // for all models

#ifdef RVLVN_PART_SEGMENTATION_TRAINING_LOG
            fclose(fpLog);
#endif

            RVLMEM_ALLOC_LOCAL_UPDATE(pMem);
            RVLMEM_ALLOC_LOCAL_FREE(pMem);

            //// END: Create model Clusters of the model template iMM of the part iLabel.

            // MST:
            MSTree MST;
            MST.Init(modelClusters.n);
            Array<GRAPH::MSTreeEdge> edges;
            edges.n = modelClusters.n * (modelClusters.n - 1) / 2;
            edges.Element = new GRAPH::MSTreeEdge[edges.n];
            GRAPH::MSTreeEdge *pEdge = edges.Element;

            for (iModelCluster1 = 0; iModelCluster1 < modelClusters.n; iModelCluster1++)
            {
                if (iModelCluster1 == iModelCluster)
                    continue;

                for (iModelCluster2 = iModelCluster1 + 1; iModelCluster2 < modelClusters.n; iModelCluster2++)
                {
                    if (iModelCluster2 == iModelCluster)
                        continue;

                    pEdge->iVertex[0] = iModelCluster1;
                    pEdge->iVertex[1] = iModelCluster2;
                    pEdge->cost = EC[iModelCluster1 * nModelClusters + iModelCluster2];

                    pEdge++;
                }
            }

            edges.n = pEdge - edges.Element;

            // MST.Create(edges, &MSTEdges, componentClusterMaxCost);
            MST.Create(edges, &MSTEdges, 0.0f);

#ifdef RVLVN_PART_SEGMENTATION_TRAINING_LOG
            MSTLogFileName[0] = '0' + iLabel;
            MSTLogFileName[1] = '0' + no;
            MSTLogFileName[2] = '0' + nu;

            fpLog = fopen((std::string(resultsFolder) + "\\" + MSTLogFileName + ".edg").data(), "w");

            for (i = 0; i < MSTEdges.n; i++)
                fprintf(fpLog, "%d\t%d\t%f\n", MSTEdges.Element[i].iVertex[0], MSTEdges.Element[i].iVertex[1], MSTEdges.Element[i].cost);

            fclose(fpLog);
#endif

            // END: MST.

            // Region growing.

#ifdef RVLVN_PART_SEGMENTATION_TRAINING_LOG
            fpLog = fopen((std::string(resultsFolder) + "\\" + MSTLogFileName).data(), "w");
#endif
            // Array2D<VN_::Gauss> *pGaussArr = &(modelTemplateVector[iLabel][iMM].gaussianMixtureModel);
            Array2D<VN_::Gauss> *pGaussArr = gauss + iOrigMMAbs;
            pGaussArr->w = nComponentsC;
            pGaussArr->h = modelClusters.n;
            gaussBlockSize = pGaussArr->w * pGaussArr->h;
            pGaussArr->Element = new VN_::Gauss[gaussBlockSize];
            // modelTemplateVector[iLabel][iMM].gaussMem = new float[gaussBlockSize * m];
            gaussMem[iOrigMMAbs] = new float[gaussBlockSize * m];

            VN_::Gauss *pGauss;

            if (iModelCluster < modelClusters.n)
            {
                for (iComp = 0; iComp < nComponentsC; iComp++)
                {
                    pGauss = pGaussArr->Element + nComponentsC * iModelCluster + iComp;

                    pGauss->w = 0.0f;
                    pGauss->x = NULL;
                    pGauss->abs = 0.0f;
                }
            }

            int *componentID = new int[nComponentsC * MST.T.NodeArray.n];

            int *nodeBuff = new int[MST.T.NodeArray.n];

            bool *bNodeVisited = new bool[MST.T.NodeArray.n];

            memset(bNodeVisited, 0, MST.T.NodeArray.n * sizeof(bool));

            bool bFirstMMCreated = false;

            int iRoot;

            for (iRoot = 0; iRoot < modelClusters.n; iRoot++)
            {
                if (iRoot == iModelCluster)
                    continue;

                if (bNodeVisited[iRoot])
                    continue;

                if (bFirstMMCreated)
                    pMetaModel = modelTemplateVector[iLabel].insert(pMetaModel + 1, modelTemplate);
                else
                    bFirstMMCreated = true;

                pMetaModel->idx = iMMAbs;

                metaModelGaussBuff.n = 0;

#ifdef RVLVN_PART_SEGMENTATION_TRAINING_LOG
                fprintf(fpLog, "%d\t", iRoot);
#endif

                int iGauss;
                VNInstance *pModelCluster;

                for (iComp = 0; iComp < nComponentsC; iComp++)
                {
                    iGauss = nComponentsC * iRoot + iComp;

                    pGauss = pGaussArr->Element + iGauss;

                    pGauss->w = (float)(clusterSize[iRoot]);
                    pGauss->x = gaussMem[iOrigMMAbs] + m * iGauss;

                    pModelCluster = modelClusters.Element[iRoot];

                    // memcpy(pGauss->x, pModelCluster->d + sampledUnitSphere.h * iComp, m * sizeof(float));
                    memcpy(pGauss->x, pModelCluster->q + m * iComp, m * sizeof(float));
                    pGauss->abs = pModelCluster->s[iComp];

                    componentID[nComponentsC * iRoot + iComp] = iComp;

#ifdef RVLVN_PART_SEGMENTATION_TRAINING_LOG
                    fprintf(fpLog, "%d\t", iComp);
#endif
                }

                metaModelGaussBuff.Element[metaModelGaussBuff.n++] = nComponentsC * iRoot;

#ifdef RVLVN_PART_SEGMENTATION_TRAINING_LOG
                fprintf(fpLog, "\n");
#endif
                int *piFetch = nodeBuff;
                int *piPush = nodeBuff;

                *(piPush++) = iRoot;

                bNodeVisited[iRoot] = true;

                int componentID_;
                GRAPH::Node *pNode;
                GRAPH::Edge *pEdge_;
                RVL::GRAPH::EdgePtr<GRAPH::Edge> *ppEdge;

                while (piFetch < piPush)
                {
                    iModelCluster1 = *(piFetch++);

                    pNode = MST.T.NodeArray.Element + iModelCluster1;

                    ppEdge = pNode->EdgeList.pFirst;

                    while (ppEdge)
                    {
                        iModelCluster2 = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(ppEdge);

                        if (!bNodeVisited[iModelCluster2])
                        {
                            bNodeVisited[iModelCluster2] = true;

#ifdef RVLVN_PART_SEGMENTATION_TRAINING_LOG
                            fprintf(fpLog, "%d\t", iModelCluster2);
#endif
                            metaModelGaussBuff.Element[metaModelGaussBuff.n++] = nComponentsC * iModelCluster2;

                            for (iComp1 = 0; iComp1 < nComponentsC; iComp1++)
                            {
                                iComp2 = RVLVN_GET_CORRESPONDENCE(correspMxC, modelClusters, iModelCluster1, iModelCluster2, iComp1);

                                componentID_ = componentID[nComponentsC * iModelCluster2 + iComp2] = componentID[nComponentsC * iModelCluster1 + iComp1];

                                iGauss = nComponentsC * iModelCluster2 + componentID_;

                                pGauss = pGaussArr->Element + iGauss;

                                pModelCluster2 = modelClusters.Element[iModelCluster2];

                                // pGauss->x = modelTemplate.gaussMem + sampledUnitSphere.h * iGauss;

                                // memcpy(pGauss->x, pModelCluster2->d + sampledUnitSphere.h * iComp2, sampledUnitSphere.h * sizeof(float));

                                pGauss->x = gaussMem[iOrigMMAbs] + m * iGauss;

                                memcpy(pGauss->x, pModelCluster2->q + m * iComp2, m * sizeof(float));
                                pGauss->abs = pModelCluster2->s[iComp2];

                                pGauss->w = (float)(clusterSize[iModelCluster2]);
                            }

#ifdef RVLVN_PART_SEGMENTATION_TRAINING_LOG
                            for (iComp2 = 0; iComp2 < nComponentsC; iComp2++)
                                fprintf(fpLog, "%d\t", componentID[nComponentsC * iModelCluster2 + iComp2]);

                            fprintf(fpLog, "\n");
#endif

                            *(piPush++) = iModelCluster2;
                        }

                        ppEdge = ppEdge->pNext;
                    }
                }

#ifdef RVLVN_PART_SEGMENTATION_TRAINING_LOG
                fclose(fpLog);
#endif
                float wTotal = 0.0f;

                for (iGauss = 0; iGauss < metaModelGaussBuff.n; iGauss++)
                {
                    pGauss = pGaussArr->Element + metaModelGaussBuff.Element[iGauss];

                    wTotal += pGauss->w;
                }

                for (iGauss = 0; iGauss < metaModelGaussBuff.n; iGauss++)
                {
                    for (iComp = 0; iComp < nComponentsC; iComp++)
                    {
                        pGauss = pGaussArr->Element + metaModelGaussBuff.Element[iGauss] + iComp;

                        pGauss->w /= wTotal;
                    }

                    metaModelGaussBuff.Element[iGauss] += nGaussTotal;
                }

                metaModelGaussArrayTemplate.Element = new int[metaModelGaussBuff.n];

                memcpy(metaModelGaussArrayTemplate.Element, metaModelGaussBuff.Element, metaModelGaussBuff.n * sizeof(int));
                metaModelGaussArrayTemplate.n = metaModelGaussBuff.n;

                metaModelGauss.push_back(metaModelGaussArrayTemplate);

                iMMAbs++;

                nMMGaussTotal += metaModelGaussBuff.n;
            } // for (iRoot = 0; iRoot < modelClusters.n; iRoot++)

            nGaussTotal += gaussBlockSize;

            delete[] edges.Element;
            // delete[] bdMem;
            // delete[] dMem;
            delete[] qMem;
            delete[] sMem;
            delete[] bConcaveMem;
            delete[] modelClusterPartComponentIdxMem;
            delete[] EC;
            delete[] correspMxC;
            delete[] correspMxCMem;
            delete[] clusterSize;
            delete[] nodeBuff;
            delete[] bNodeVisited;
            delete[] componentID;

            for (iModelCluster = 0; iModelCluster < modelClusters.n; iModelCluster++)
            {
                modelClusters.Element[iModelCluster]->bConcave = NULL;
                modelClusters.Element[iModelCluster]->q = NULL;
                modelClusters.Element[iModelCluster]->s = NULL;
            }

            modelCluster.bConcave = NULL;
            modelCluster.q = NULL;
            modelCluster.s = NULL;

            iOrigMMAbs++;

            //	MST.Create(edges[iEdge]);
            //	struct correspondanceVariation
            //	{
            //		int iModel;
            //		std::vector<int> components;
            //		int ID;
            //	};
            //	std::vector<correspondanceVariation> correspondanceVariationArray;
            //	correspondanceVariation correspondanceVariationTemp;
            //	MSTree MST2;
            //	MST2.Init(10000);
            //	iEdge = 0;
            //	int M1, M2, id = 0;
            //	int *corresp;
            //	for (int iNode = 0; iNode < MST.T.NodeArray.n; iNode++)
            //	{
            //		RVL::GRAPH::EdgePtr<RVL::GRAPH::Edge> *ppEdge = MST.T.NodeArray.Element[iNode].EdgeList.pFirst;
            //		while (ppEdge)
            //		{
            //			M1 = ppEdge->pEdge->iVertex[0];
            //			M2 = ppEdge->pEdge->iVertex[1];
            //			int nComponents = modelVNArray.Element[M1]->nComponents;
            //			corresp = correspMatrix[M1].Element + nComponents*M2;
            //			for (int iComponent = 0; iComponent < nComponents; iComponent++)
            //			{
            //				int correspondence = corresp[iComponent];
            //				if (correspondence != -1)
            //				{
            //					correspondanceVariationTemp.components.push_back(iComponent);
            //				}
            //			}
            //			//check if exist
            //			correspondanceVariationTemp.iModel = M1;
            //			correspondanceVariationTemp.ID = id;
            //			correspondanceVariationArray.push_back(correspondanceVariationTemp);
            //			correspondanceVariationTemp.components.clear();
            //			id++;
            //			iEdge++;
            //		}
            //		//edges[iEdge].Element->iVertex[0] =
            //		//edges[iEdge].Element->iVertex[1] =
            //		//greedy:
            //		edges[iEdge].Element->cost = E[(iModel1*nModels + iModel2)*nLabels + iLabel];
            //		ppEdge->pNext;
            //	}
            // no = modelTemplateVector[iLabel][iMM].no;
            // nu = modelTemplateVector[iLabel][iMM].nu;
            // int nComponents = no + nu;
            // Pair<int, int> *buff = new Pair<int, int>[nModels];
            // int iFirstOComponent = 0;
            // int iFirstUComponent = 0;
            // Pair<int, int> *piPush;
            // Pair<int, int> *piFetch;
            // int iComponent;
            // for (iComponent = 0; iComponent < nComponents; iComponent++)
            //{
            //	//Region growing:
            //	//Mcorrespondences = modelTemplateVector[iLabel][iMM].correspondences.pFirst;
            //	while ()
            //	{
            //	}
            //		if (iComponent < no)
            //			piPush = buff;
            //	piFetch = buff;
            //	pVNModelInstance = modelVNArray.Element[MST.T.NodeArray.Element[0].idx];
            //	piPush->a = 0;
            //	//piPush->b =
            //	*piPush++;
            //	//std::vector<int> nComponentsPerComponentCluster;
            //	//std::vector<int> componentClusterLabel;
            //	//std::vector<bool> componentClusterConcavity;
            //	//fPrimitiveClass = fopen((std::string(resultsFolder) + "\\primitiveClass.txt").data(), "a");
            //	int i, nComponentClusters = 0;
            //	int nComponentsInCluster;
            //	while (piFetch < piPush)		// region growing loop
            //	{
            //		comp = *(piFetch++);
            //		GRAPH::EdgePtr<GRAPH::Edge> *pEdgePtr = MST.T.NodeArray.Element[comp.a].EdgeList.pFirst;
            //		i = 0;
            //		while (pEdgePtr)
            //		{
            //			comp_.a = (comp.a == pEdgePtr->pEdge->iVertex[0] ? pEdgePtr->pEdge->iVertex[1] : pEdgePtr->pEdge->iVertex[0]);
            // 		comp_.b = correspMatrix[comp.a].Element[modelVNArray.Element[comp.a]->nComponents*comp_.a + comp.b];
            //			if (comp_.b >= 0)
            //			{
            //				if (modelVNArray.Element[comp_.a]->iPrimitiveClass[comp_.b] < 0)
            //				{
            //					if (comp_.a == 0)
            //						int debug = 0;
            //					*(piPush++) = comp_;
            //					modelVNArray.Element[comp_.a]->iPrimitiveClass[comp_.b] = nComponentClusters;
            //					modelVNArray.Element[comp.a]->iPrimitiveClass[comp.b] = nComponentClusters;
            //				}
            //			}
            //			pEdgePtr = pEdgePtr->pNext;
            //		}
            //		nComponentsInCluster = piFetch - buff;
            //		if (nComponentsInCluster > 1)
            //		{
            //			nComponentsPerComponentCluster.push_back(nComponentsInCluster);
            //			componentClusterLabel.push_back(modelVNArray.Element[comp.a]->label[comp.b]);
            //			componentClusterConcavity.push_back(modelVNArray.Element[comp.a]->bConcave[comp.b]);
            //			nComponentClusters++;
            //		}
            //	}
            //}
            // delete[] buff;
            // RVL_DELETE_ARRAY(componentClusters.Element);
            // componentClusters.Element = new RECOG::VN_::ComponentCluster[nComponentClusters];
            // componentClusters.n = nComponentClusters;
        } // for every metamodel
    }     // for every label

    int nMMsTotal = iMMAbs;

    printf("\nModel clusters created.\n\n");

#ifdef RVLVN_PART_SEGMENTATION_TRAINING_LOG
    fclose(fpLMM);
#endif

    ///// END: Create model clusters.

    // Copy gaussians to classes[0].gauss.

    RVL_DELETE_ARRAY(classes[0].gauss.Element);

    classes[0].gauss.Element = new VN_::Gauss[nGaussTotal];
    classes[0].gauss.n = nGaussTotal;

    RVL_DELETE_ARRAY(classes[0].gaussMem);

    classes[0].gaussMem = new float[m * nGaussTotal];

    VN_::Gauss *pGaussBlock = classes[0].gauss.Element;

    float *pGaussMem = classes[0].gaussMem;

    //	int debug = 0;

    for (iOrigMMAbs = 0; iOrigMMAbs < nOrigMMsTotal; iOrigMMAbs++)
    {
        gaussBlockSize = gauss[iOrigMMAbs].w * gauss[iOrigMMAbs].h;

        memcpy(pGaussBlock, gauss[iOrigMMAbs].Element, gaussBlockSize * sizeof(VN_::Gauss));

        pGaussBlock += gaussBlockSize;

        memcpy(pGaussMem, gaussMem[iOrigMMAbs], gaussBlockSize * m * sizeof(float));

        pGaussMem += (gaussBlockSize * m);

        delete[] gauss[iOrigMMAbs].Element;
        delete[] gaussMem[iOrigMMAbs];

        //		debug += gaussBlockSize;
    }

    delete[] gauss;
    delete[] gaussMem;

    // Copy metaModelGauss to classes[0].metaModelGauss.

    RVL_DELETE_ARRAY(classes[0].metaModelGauss);

    classes[0].metaModelGauss = new Array<int>[nMMsTotal];

    RVL_DELETE_ARRAY(classes[0].metaModelGaussMem);

    classes[0].metaModelGaussMem = new int[nMMGaussTotal];

    int *pGaussIdxArray = classes[0].metaModelGaussMem;

    for (iMMAbs = 0; iMMAbs < nMMsTotal; iMMAbs++)
    {
        memcpy(pGaussIdxArray, metaModelGauss[iMMAbs].Element, metaModelGauss[iMMAbs].n * sizeof(int));
        classes[0].metaModelGauss[iMMAbs].Element = pGaussIdxArray;
        classes[0].metaModelGauss[iMMAbs].n = metaModelGauss[iMMAbs].n;
        pGaussIdxArray += metaModelGauss[iMMAbs].n;
        delete[] metaModelGauss[iMMAbs].Element;
    }

    // Assign relative metamodel addresses to their absolute addresses.

    iMMAbs = 0;

    classes[0].metamodelID.n = nMMsTotal;

    RVL_DELETE_ARRAY(classes[0].metamodelID.Element);

    classes[0].metamodelID.Element = new Pair<int, int>[nMMsTotal];

    for (iLabel = 0; iLabel < nLabels; iLabel++)
    {
        iMM = 0;

        for (pMetaModel = modelTemplateVector[iLabel].begin(); pMetaModel != modelTemplateVector[iLabel].end(); ++pMetaModel, iMM++)
        {
            classes[0].metamodelID.Element[iMMAbs].a = iLabel;
            classes[0].metamodelID.Element[iMMAbs].b = iMM;
        }
    }

    // Free memory.

    delete[] E;
    // delete[] modelTemplateVector;
    // delete[] modelTemplateVector[iModel].bComponentCluster
    delete[] correspMatrix;
    delete[] correspMem;
    delete[] modelTemplateCorrespListMem;
    delete[] modelClusters.Element;
    delete[] modelClustersMem;
    delete[] ModelTemplateInstanceListArr.Element;
    delete[] components1[0].Element;
    delete[] components1[1].Element;
    delete[] modelClusterPartMem;
    delete[] metaModelGaussBuff.Element;
    delete[] MSTEdges.Element;

    //------------------------
    // old version

    // if (bMST)
    //{
    //	MSTree MST;
    //	MST.Init(10000);
    //	int nNodes;
    //	VNInstance *pVNModelInstance;
    //	pVNModelInstance = pModelVNList->pFirst;
    //	int nModels = 0;
    //	int nMComponents = 0;
    //	while (pVNModelInstance)
    //	{
    //		nMComponents += pVNModelInstance->nComponents;
    //		nModels++;
    //		pVNModelInstance = pVNModelInstance->pNext;
    //	}
    //	Array<int> *correspMatrix = new Array<int>[nModels];
    //	int *correspMem = new int[nModels*nMComponents];
    //	int *pCorrespMatrix = correspMem;
    //	int iModel1 = 0;
    //	pVNModelInstance = pModelVNList->pFirst;
    //	RVL_DELETE_ARRAY(modelVNArray.Element);
    //	modelVNArray.Element = new VNInstance *[nModels];
    //	QLIST::CreatePtrArray<VNInstance>(pModelVNList, &modelVNArray);
    //	//float *EMem = new float[modelVNArray.n * (modelVNArray.n - 1) / 2];
    //	//float *E = EMem;
    //	float *E;
    //	while (pVNModelInstance)
    //	{
    //		//printf("%d/%d\n", iModel1, modelVNArray.n);
    //		correspMatrix[iModel1].Element = pCorrespMatrix;
    //		E = (MST.T.NodeArray.n > 0 ? new float[MST.T.NodeArray.n] : NULL);
    //		if (iModel1 == 842)
    //			int debug = 0;
    //		int *Correspondences;
    //		if (iModel1 > 0)
    //		{
    //			pVNModelInstance->Match(pModelVNList, Correspondences, this, &mesh, E, correspMatrix, iModel1);
    //			delete[] Correspondences;
    //		}
    //		MST.Update(E);
    //
    //		//E += iModel1;
    //		pCorrespMatrix += (nModels * pVNModelInstance->nComponents);
    //		pVNModelInstance->iPrimitiveClass = new int[pVNModelInstance->nComponents];
    //		memset(pVNModelInstance->iPrimitiveClass, 0xff, pVNModelInstance->nComponents*sizeof(int));
    //		iModel1++;
    //
    //		pVNModelInstance = pVNModelInstance->pNext;
    //	}
    //	//Region growing:
    //	pVNModelInstance = pModelVNList->pFirst;
    //	Pair<int, int> *buff = new Pair<int, int>[nModels];
    //	Pair<int, int> *piPush;
    //	Pair<int, int> *piFetch;
    //	Pair<int, int> comp, comp_;
    //	std::vector<int> nComponentsPerComponentCluster;
    //	std::vector<int> componentClusterLabel;
    //	std::vector<bool> componentClusterConcavity;
    //	fPrimitiveClass = fopen((std::string(resultsFolder) + "\\primitiveClass.txt").data(), "a");
    //	int i, nComponentClusters = 0;
    //	int nComponentsInCluster;
    //	for(int iSeedModel = 0; iSeedModel < nModels; iSeedModel++)
    //	{
    //		for (int iSeedComp = 0; iSeedComp < pVNModelInstance->nComponents; iSeedComp++)
    //		{
    //			comp.a = iSeedModel;
    //			comp.b = iSeedComp;
    //			if (modelVNArray.Element[comp.a]->iPrimitiveClass[comp.b] >= 0)
    //				continue;
    //			piPush = buff;
    //			piFetch = buff;
    //			*(piPush++) = comp;
    //			while (piFetch < piPush)		// region growing loop
    //			{
    //
    //				comp = *(piFetch++);
    //				GRAPH::EdgePtr<GRAPH::Edge> *pEdgePtr = MST.T.NodeArray.Element[comp.a].EdgeList.pFirst;
    //				i = 0;
    //				while (pEdgePtr)
    //				{
    //					comp_.a = (comp.a == pEdgePtr->pEdge->iVertex[0] ? pEdgePtr->pEdge->iVertex[1] : pEdgePtr->pEdge->iVertex[0]);
    //					comp_.b = correspMatrix[comp.a].Element[modelVNArray.Element[comp.a]->nComponents*comp_.a + comp.b];
    //					if (comp_.b >= 0)
    //					{
    //						if (modelVNArray.Element[comp_.a]->iPrimitiveClass[comp_.b] < 0)
    //						{
    //							if (comp_.a == 0)
    //								int debug = 0;
    //							*(piPush++) = comp_;
    //							modelVNArray.Element[comp_.a]->iPrimitiveClass[comp_.b] = nComponentClusters;
    //							modelVNArray.Element[comp.a]->iPrimitiveClass[comp.b] = nComponentClusters;
    //						}
    //					}
    //					pEdgePtr = pEdgePtr->pNext;
    //				}
    //			}	// region growing loop
    //			nComponentsInCluster = piFetch - buff;
    //			if (nComponentsInCluster > 1)
    //			{
    //				nComponentsPerComponentCluster.push_back(nComponentsInCluster);
    //				componentClusterLabel.push_back(modelVNArray.Element[comp.a]->label[comp.b]);
    //				componentClusterConcavity.push_back(modelVNArray.Element[comp.a]->bConcave[comp.b]);
    //				nComponentClusters++;
    //			}
    //		}
    //		pVNModelInstance = pVNModelInstance->pNext;
    //	}
    //	RVL_DELETE_ARRAY(componentClusters.Element);
    //	componentClusters.Element = new RECOG::VN_::ComponentCluster [nComponentClusters];
    //	componentClusters.n = nComponentClusters;
    //
    //	for (int iCompCluster = 0; iCompCluster < componentClusters.n; iCompCluster++)
    //	{
    //		componentClusters.Element[iCompCluster].label = componentClusterLabel[iCompCluster];
    //		componentClusters.Element[iCompCluster].bConcave = componentClusterConcavity[iCompCluster];
    //		componentClusters.Element[iCompCluster].nComponents = nComponentsPerComponentCluster[iCompCluster];
    //	}
    //	std::vector<RECOG::VN_::ModelTemplate> modelTemplateVector;
    //	RECOG::VN_::ModelTemplate modelTemplate;
    //	int nActiveComponents;
    //	for (int iModel = 0; iModel < nModels; iModel++)
    //	{
    //		for (int iComp = 0; iComp < modelVNArray.Element[iModel]->nComponents; iComp++)
    //		{
    //			fprintf(fPrimitiveClass, "%d\t%d\t%d\n", iModel, iComp, modelVNArray.Element[iModel]->iPrimitiveClass[iComp]);
    //		}
    //	}
    //	fclose(fPrimitiveClass);
    //	for (int iModel = 0; iModel < nModels; iModel++)
    //	{
    //		bool *bComponentCluster_ = new bool[nComponentClusters*sizeof(bool)];
    //		memset(bComponentCluster_, 0, nComponentClusters);
    //		nActiveComponents = 0;
    //		for (int iComp = 0; iComp < modelVNArray.Element[iModel]->nComponents; iComp++)
    //		{
    //			if (modelVNArray.Element[iModel]->iPrimitiveClass[iComp] != -1)
    //			{
    //				if (bComponentCluster_[modelVNArray.Element[iModel]->iPrimitiveClass[iComp]] == 0)
    //				{
    //					bComponentCluster_[modelVNArray.Element[iModel]->iPrimitiveClass[iComp]] = 1;
    //					nActiveComponents++;
    //				}
    //			}
    //		}
    //		//check if this modelTemplate already exists:
    //		bool exists = 0;
    //		int nCC;
    //		bool bCCTrue = true;
    //		for (int iCC = 0; iCC < modelTemplateVector.size(); iCC++)
    //		{
    //			for (int ibCC = 0; ibCC < nComponentClusters; ibCC++)
    //			{
    //				if (bComponentCluster_[ibCC] != modelTemplateVector[iCC].bComponentCluster[ibCC])
    //				{
    //					bCCTrue = false;
    //					break;
    //				}
    //			}
    //			if (bCCTrue)
    //			{
    //				exists = 1;
    //				nCC = iCC;
    //				break;
    //			}
    //		}
    //		if (exists == 1)
    //		{
    //			modelTemplateVector[nCC].f++;
    //		}
    //		else
    //		{
    //			modelTemplate.iComponentCluster.Element = new int[nActiveComponents];
    //			modelTemplate.iComponentCluster.n = 0;
    //			modelTemplate.bComponentCluster = new bool[nComponentClusters*sizeof(bool)];
    //			memset(modelTemplate.bComponentCluster, 0, nComponentClusters);
    //			modelTemplate.f = 1;
    //			for (int iComp = 0; iComp < modelVNArray.Element[iModel]->nComponents; iComp++)
    //			{
    //				if (modelTemplate.bComponentCluster[iComp] = bComponentCluster_[iComp])
    //					modelTemplate.iComponentCluster.Element[modelTemplate.iComponentCluster.n++] = iComp;
    //			}
    //			modelTemplateVector.push_back(modelTemplate);
    //		}
    //		delete[] bComponentCluster_;
    //	}
    //	//delete[] modelTemplateVector;
    //	//delete[] modelTemplateVector[iModel].bComponentCluster
    //	delete[] correspMatrix;
    //	delete[] correspMem;
    // }
    //------------------------
}

void VNClassifier::CreateComponentAssociationGraph()
{
    // Parameters.

    int nNN = 100;

    // Allocate arrays.

    nMCompsTotalOU[0] = nMCompsTotalOU[1] = 0;

    int nCompsTotal = 0;
    int maxnComps = 0;

    int iModel, iMComp, ou;
    VNInstance *pModel;

    for (iModel = 0; iModel < modelVNArray.n; iModel++)
    {
        pModel = modelVNArray.Element[iModel];

        nCompsTotal += pModel->nComponents;

        if (pModel->nComponents > maxnComps)
            maxnComps = pModel->nComponents;

        for (iMComp = 0; iMComp < pModel->nComponents; iMComp++)
        {
            ou = (pModel->bConcave[iMComp] ? 1 : 0);

            nMCompsTotalOU[ou]++;
        }
    }

    VN_::CTINetOutput CTINet;

    CTINet.fComp = new float[nCompsTotal * maxnComps];
    CTINet.fShape = new float[nCompsTotal * maxnComps];
    CTINet.fSize = new float[nCompsTotal * maxnComps];
    CTINet.fNeighborhood = new float[nCompsTotal * maxnComps];
    CTINet.fModel = new float[modelVNArray.n * modelVNArray.n];

    componentAssociationGraph.Clear();

    componentAssociationGraph.EdgeArray.n = 0;
    componentAssociationGraph.EdgeArray.Element = new VN_::CAGEdge[nNN * nCompsTotal];

    Array<VN_::CAGEdge> edges = componentAssociationGraph.EdgeArray;

    // If a component association file exists, then load the component associations.

    char *associationFileName = RVLCreateFileName(modelDataBase, ".dat", -1, ".cag");

    // FILE *fpAssociations = fopen(associationFileName, "rb");
    FILE *fpAssociations = NULL;

    int iMCompAbs, iEdge, iTmp;
    VN_::CAGEdge *pEdge;
    int iVertex[2];
    float edgeCost;

    if (fpAssociations)
    {
        while (!feof(fpAssociations))
        {
            iTmp = fread(iVertex, sizeof(int), 2, fpAssociations);
            if (iTmp < 2)
                break;
            fread(&edgeCost, sizeof(float), 1, fpAssociations);
            pEdge = edges.Element + edges.n++;
            pEdge->iVertex[0] = iVertex[0];
            pEdge->iVertex[1] = iVertex[1];
            pEdge->cost = edgeCost;
        }

        fclose(fpAssociations);
    }
    else
    {
        VN_::PartAssociation *componentAssociation, *cellAssociation;

#ifdef RVLVN_PART_SEGMENTATION_CTINET_PROBABILISTIC
        // Compute prior probability for all components.

        printf("Compute prior probability for all components...\n");

        float *lnPDFPrior = new float[nCompsTotal];

        iMCompAbs = 0;

        for (iModel = 0; iModel < modelVNArray.n; iModel++)
        {
            pModel = modelVNArray.Element[iModel];

            pModel->CTINet(this, 0, componentAssociation, cellAssociation, nMCompsTotalOU, alphaCTINetPrior, true, false, false, true, false, lnPDFPrior + iMCompAbs, fComp, fModel, modelVNArray.n);

            iMCompAbs += pModel->nComponents;

            printf("%d%%\r", 100 * iModel / modelVNArray.n);
        }

        printf("100%%\n");
#endif

        // Match all models with eachother using CTINet.

        FILE *fpModels;

#ifdef RVLVN_PART_SEGMENTATION_CTINET_PCFIT
        fpModels = fopen(modelLayerFileName, "rb");

        if (fpModels)
        {
            fread(CTINet.fModel, sizeof(float), modelVNArray.n * modelVNArray.n, fpModels);

            fclose(fpModels);
        }
        else
        {
            printf("Matching all models with eachother...\n");

            fpModels = fopen((std::string(resultsFolder) + "\\VNInstanceToClassModelLayer.dat").data(), "wb");

            fclose(fpModels);

            for (iModel = 0; iModel < modelVNArray.n; iModel++)
            {
                fpModels = fopen((std::string(resultsFolder) + "\\VNInstanceToClassModelLayer.dat").data(), "ab");

                printf("Matching model %d with the other models...\n", iModel);

                pModel = modelVNArray.Element[iModel];

                pModel->CTINet(this, 0, CTINet, componentAssociation, cellAssociation, nMCompsTotalOU, alphaCTINetPrior, RVLVN_CTINET_MODEL_LAYER, NULL, modelVNArray.n, iModel);

                printf("%d%%\r", 100 * iModel / modelVNArray.n);

                fwrite(CTINet.fModel, sizeof(float), modelVNArray.n, fpModels);

                fclose(fpModels);
            }

            printf("100%%\n");
        }
#else
#ifdef RVLVN_PART_SEGMENTATION_TRAINING_LOG
        fpModels = fopen(modelLayerFileName, "wb");
#endif
#endif
        int i, iModel_;
        VNInstance *pModel_;

        // pModel = modelVNArray.Element[iRefModel];

        // float maxfModel = 0.0f;

        // float fModel_;

        // for (i = 0; i < modelSet.n; i++)
        //{
        //	iModel = modelSet.Element[i];

        //	fModel_ = fModel[iRefModel * modelVNArray.n + iModel] * fModel[iModel * modelVNArray.n + iRefModel];

        //	if (fModel_ > maxfModel)
        //	{
        //		maxfModel = fModel_;

        //		iModel_ = iModel;
        //	}
        //}

        // pModel_ = modelVNArray.Element[iModel_];

        // float *q;

        // pModel->Fit(this, pModel_, componentAssociationAlphat, componentAssociationAlphas, componentAssociationSig, q);

        // delete[] q;

        printf("Create the compontnt association graph...\n");

        Array<VN_::CAGEdge> edgeBuff;
        edgeBuff.Element = new VN_::CAGEdge[nCompsTotal];

        Array<VN_::CAGEdge> edges_;
        edges_.Element = edges.Element;

        int iEdge = 0;

        int j;
        int iMComp_, iMCompAbs_;
        bool bConcave;
        float fComp, fModel, fNeighborhood;

        for (i = 0; i < modelSet.n; i++)
        {
            iModel = modelSet.Element[i];

            pModel = modelVNArray.Element[iModel];

#ifdef RVLVN_PART_SEGMENTATION_CTINET_PCFIT
            pModel->CTINet(this, 0, CTINet, componentAssociation, cellAssociation, nMCompsTotalOU, alphaCTINetPrior,
                           RVLVN_CTINET_COMPONENT_LAYER | RVLVN_CTINET_NEIGHBORHOOD_LAYER, NULL, modelVNArray.n, iModel);
#else
#ifdef RVLVN_PART_SEGMENTATION_TRAINING_LOG
            fpModels = fopen((std::string(resultsFolder) + "\\VNInstanceToClassModelLayer.dat").data(), "ab");
#endif

            printf("Matching model %d with the other models...\n", iModel);

#ifdef RVLVN_PART_SEGMENTATION_CTINET_PROBABILISTIC
            pModel->CTINet(this, 0, componentAssociation, cellAssociation, nMCompsTotalOU, alphaCTINetPrior, true, true, false, false, false, lnPDFPrior, fComp, fModel, modelVNArray.n, iModel);
#else
            pModel->CTINet(this, 0, componentAssociation, cellAssociation, nMCompsTotalOU, alphaCTINetPrior, true, true, false, false, false, NULL, fComp, fModel, modelVNArray.n, iModel);
#endif
#endif
            iMCompAbs = firstComponentAbsIdx.Element[iModel];

            for (iMComp = 0; iMComp < pModel->nComponents; iMComp++, iMCompAbs++)
            {
                bConcave = pModel->bConcave[iMComp];

                edgeBuff.n = 0;

                for (j = 0; j < modelSet.n; j++)
                {
                    iModel_ = modelSet.Element[j];

                    pModel_ = modelVNArray.Element[iModel_];

                    if (iModel_ == iModel)
                        continue;

                    iMCompAbs_ = firstComponentAbsIdx.Element[iModel_];

                    for (iMComp_ = 0; iMComp_ < pModel_->nComponents; iMComp_++, iMCompAbs_++)
                    {
                        if (pModel_->bConcave[iMComp_] == bConcave)
                        {
                            fComp = CTINet.fComp[iMComp * nCompsTotal + iMCompAbs_];

                            if (fComp < componentAssociationfCompThr)
                                continue;

                            fNeighborhood = CTINet.fNeighborhood[iMComp * nCompsTotal + iMCompAbs_];
                            fModel = CTINet.fModel[iModel * modelVNArray.n + iModel_] * CTINet.fModel[iModel_ * modelVNArray.n + iModel];

                            pEdge = edgeBuff.Element + edgeBuff.n;

                            pEdge->iVertex[0] = iMCompAbs;
                            pEdge->iVertex[1] = iMCompAbs_;
#ifdef RVLVN_PART_SEGMENTATION_CTINET_PROBABILISTIC
                            pEdge->cost = fModel[iModel_] + fComp[iMComp * nCompsTotal + iMCompAbs_];
#else
#ifdef RVLVN_PART_SEGMENTATION_CTINET_PCFIT
                            // pEdge->cost = -CTINet.fModel[iModel * modelVNArray.n + iModel_] * CTINet.fModel[iModel_ * modelVNArray.n + iModel] * CTINet.fComp[iMComp * nCompsTotal + iMCompAbs_];

                            pEdge->cost = -(fComp + wNeighborhoodSimilarity * fNeighborhood + wModelSimilarity * fModel);
                            // pEdge->cost = -(fComp + wNeighborhoodSimilarity * fNeighborhood) * fModel;
                            // pEdge->cost = -(fComp * fModel);
                            pEdge->fComp = fComp;
                            pEdge->fNeighborhood = fNeighborhood;
                            pEdge->fModel = fModel;
#else
                            pEdge->cost = -fModel[iModel_] * fComp[iMComp * nCompsTotal + iMCompAbs_];
#endif
#endif

                            edgeBuff.n++;
                        }
                    }
                }

                if (edgeBuff.n > 0)
                {
                    Min<VN_::CAGEdge, float>(edgeBuff, RVLMIN(nNN, edgeBuff.n), edges_);

                    edges_.Element += edges_.n;
                    edges.n += edges_.n;
                }
            }

            printf("%d%%\r", 100 * iModel / modelVNArray.n);

#ifndef RVLVN_PART_SEGMENTATION_CTINET_PCFIT
#ifdef RVLVN_PART_SEGMENTATION_TRAINING_LOG
            fwrite(fModel, sizeof(float), modelVNArray.n, fpModels);

            fclose(fpModels);
#endif
#endif
        }

        delete[] edgeBuff.Element;
#ifdef RVLVN_PART_SEGMENTATION_CTINET_PROBABILISTIC
        delete[] lnPDFPrior;
#endif

        printf("100%%\n");
    } // if the association file doesn't exist

    componentAssociationGraph.EdgeArray.n = edges.n;

    // Create the component association graph.

    componentAssociationGraph.Create(nCompsTotal);

    if (fpAssociations == NULL)
    {
        // Save component associations to a file.

        fpAssociations = fopen(associationFileName, "wb");

        GRAPH::Node_<GRAPH::EdgePtr<VN_::CAGEdge>> *pCAGNode;
        GRAPH::EdgePtr<VN_::CAGEdge> *pEdgePtr;

        for (iMComp = 0; iMComp < componentAssociationGraph.NodeArray.n; iMComp++)
        {
            pCAGNode = componentAssociationGraph.NodeArray.Element + iMComp;

            pEdgePtr = pCAGNode->EdgeList.pFirst;

            while (pEdgePtr)
            {
                if (pEdgePtr->pEdge->iVertex[0] == iMComp)
                {
                    fwrite(pEdgePtr->pEdge->iVertex, sizeof(int), 2, fpAssociations);
                    fwrite(&(pEdgePtr->pEdge->cost), sizeof(float), 1, fpAssociations);
                    fwrite(&(pEdgePtr->pEdge->fComp), sizeof(float), 1, fpAssociations);
                    fwrite(&(pEdgePtr->pEdge->fNeighborhood), sizeof(float), 1, fpAssociations);
                    fwrite(&(pEdgePtr->pEdge->fModel), sizeof(float), 1, fpAssociations);
                }

                pEdgePtr = pEdgePtr->pNext;
            }
        }

        fclose(fpAssociations);
    }

    delete[] associationFileName;

    // Create model MST.

    // edges.n = modelVNArray.n * (modelVNArray.n - 1) / 2;
    // edges.Element = new GRAPH::MSTreeEdge[edges.n];
    // pEdge = edges.Element;
    // Array<GRAPH::MSTreeEdge> MSTEdges;
    // MSTEdges.Element = new GRAPH::MSTreeEdge[modelVNArray.n - 1];

    // int iModel_;

    // for (iModel = 1; iModel < modelVNArray.n; iModel++)
    //{
    //	for (iModel_ = 0; iModel_ < iModel; iModel_++)
    //	{
    //		pEdge->iVertex[0] = iModel;
    //		pEdge->iVertex[1] = iModel_;
    //		pEdge->cost = -fModel[iModel * modelVNArray.n + iModel_];

    //		pEdge++;
    //	}
    //}

    // edges.n = pEdge - edges.Element;

    delete[] CTINet.fComp;
    delete[] CTINet.fShape;
    delete[] CTINet.fSize;
    delete[] CTINet.fNeighborhood;
    delete[] CTINet.fModel;
}

// Move to Util.h

template <typename EntryType, typename CostType>
bool SortedListUpdate(
    Array<QList<EntryType>> Q,
    CostType binSize,
    EntryType *pEntry)
{
    int idx = (int)floor(pEntry->cost / binSize);

    if (idx < 0 || idx >= Q.n)
        return false;

    QList<EntryType> *pQBin = Q.Element + idx;

    EntryType **ppPrevEntry = &(pQBin->pFirst);
    EntryType *pNextEntry = *ppPrevEntry;

    while (pNextEntry)
    {
        if (pNextEntry->cost > pEntry->cost)
            break;

        ppPrevEntry = &(pNextEntry->pNext);

        pNextEntry = *ppPrevEntry;
    }

    RVLQLIST_INSERT_ENTRY2_(ppPrevEntry, pEntry);

    return true;
}

template <typename T>
struct SortIndexListEntry
{
    int idx;
    T cost;
    SortIndexListEntry<T> *pNext;
    SortIndexListEntry<T> **pPtrToThis;
};

void VNClassifier::ComponentAssociation(float *fLabel)
{
    // Parameters.

    int nQBins = 1000000;

    // Constants.

    float binSize = 1.0f / (float)nQBins;

    // Read representatives from file.

    FILE *fpRepresentatives = fopen(representativesFileName, "r");

    Array<Pair<int, int>> representatives;

    representatives.n = 0;

    int iRefModel, iRefComp, readCount;

    while (!feof(fpRepresentatives))
    {
        readCount = fscanf(fpRepresentatives, "%d\t%d\n", &iRefModel, &iRefComp);

        if (readCount == 2)
            representatives.n++;
        else
            break;
    }

    fclose(fpRepresentatives);

    representatives.Element = new Pair<int, int>[representatives.n];

    fpRepresentatives = fopen(representativesFileName, "r");

    int iRepresentative = 0;

    while (!feof(fpRepresentatives))
    {
        readCount = fscanf(fpRepresentatives, "%d\t%d\n", &iRefModel, &iRefComp);

        if (readCount == 2)
        {
            representatives.Element[iRepresentative].a = iRefModel;
            representatives.Element[iRepresentative].b = iRefComp;
            iRepresentative++;
        }
        else
            break;
    }

    fclose(fpRepresentatives);

    //

    int nMCompsTotal = nMCompsTotalOU[0] + nMCompsTotalOU[1];

    Array<QList<SortIndexListEntry<float>>> Q;

    Q.Element = new QList<SortIndexListEntry<float>>[nQBins];
    Q.n = nQBins;

    int i;
    QList<SortIndexListEntry<float>> *pQBin;

    for (i = 0; i < Q.n; i++)
    {
        pQBin = Q.Element + i;

        RVLQLIST_INIT(pQBin);
    }

    SortIndexListEntry<float> *QEntry = new SortIndexListEntry<float>[nMCompsTotal];

    memset(fLabel, 0, nMCompsTotal * sizeof(float));

    float *w = new float[nMCompsTotal];

    memset(w, 0, nMCompsTotal * sizeof(float));

    int iMCompAbs, iMComp;
    VNInstance *pModel;
    SortIndexListEntry<float> *pQEntry;

    for (iRepresentative = 0; iRepresentative < representatives.n; iRepresentative++)
    {
        iRefModel = representatives.Element[iRepresentative].a;
        iRefComp = representatives.Element[iRepresentative].b;

        pModel = modelVNArray.Element[iRefModel];

        iMCompAbs = firstComponentAbsIdx.Element[iRefModel];

        pQBin = Q.Element;

        for (iMComp = 0; iMComp < pModel->nComponents; iMComp++, iMCompAbs++)
        {
            fLabel[iMCompAbs] = (iMComp == iRefComp ? 1.0f : -1.0f);

            w[iMCompAbs] = 1.0f;

            pQEntry = QEntry + iMCompAbs;

            pQEntry->idx = iMCompAbs;
            pQEntry->cost = 0.0f;

            RVLQLIST_ADD_ENTRY2(pQBin, pQEntry);
        }
    }

    int count = 0;

    QList<GRAPH::EdgePtr<VN_::CAGEdge>> *pEdgeList;
    GRAPH::Node_<GRAPH::EdgePtr<VN_::CAGEdge>> *pNode;
    GRAPH::EdgePtr<VN_::CAGEdge> *pEdgePtr;
    int iMCompAbs_, iBin;
    float eLabel, eLabel_, w_, fLabelSrc;
    bool bUpdateQ;
    QList<SortIndexListEntry<float>> *pQBin_;
    SortIndexListEntry<float> *pQEntry_;

    for (iBin = 0; iBin < nQBins; iBin++)
    {
        pQBin = Q.Element + iBin;

        pQEntry = pQBin->pFirst;

        while (pQEntry)
        {
            iMCompAbs = pQEntry->idx;

            count++;

            if (count % 100 == 0)
                printf("%d%%\r", 100 * count / nMCompsTotal);

            fLabelSrc = fLabel[iMCompAbs];

            eLabel = QEntry[iMCompAbs].cost;

            pNode = componentAssociationGraph.NodeArray.Element + iMCompAbs;

            pEdgeList = &(pNode->EdgeList);

            pEdgePtr = pEdgeList->pFirst;

            while (pEdgePtr)
            {
                iMCompAbs_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr);

                pQEntry_ = QEntry + iMCompAbs_;

#ifdef RVLVN_PART_SEGMENTATION_CTINET_PCFIT
                w_ = pow(-pEdgePtr->pEdge->cost, componentAssociationBeta);
#else
                w_ = exp(-(pEdgePtr->pEdge->cost > -87.0 ? pEdgePtr->pEdge->cost : -87.0));
#endif

                // if (!isfinite(w_))
                //	int debug = 0;

                if (w_ > 0.0f)
                {
                    if (w[iMCompAbs_] > 0.0f)
                    {
                        eLabel_ = pQEntry_->cost;

                        if (bUpdateQ = (eLabel_ > eLabel))
                        {
                            pQBin_ = Q.Element + (int)floor(eLabel_ / binSize);

                            RVLQLIST_REMOVE_ENTRY2(pQBin_, pQEntry_, SortIndexListEntry<float>);
                        }
                    }
                    else
                    {
                        bUpdateQ = true;

                        pQEntry_->idx = iMCompAbs_;
                    }

                    if (bUpdateQ)
                    {
                        fLabel[iMCompAbs_] = fLabel[iMCompAbs_] * w[iMCompAbs_] + w_ * componentAssociationGamma * fLabelSrc;
                        w[iMCompAbs_] += w_;
                        fLabel[iMCompAbs_] /= w[iMCompAbs_];
                        pQEntry_->cost = 1.0f - RVLABS(fLabel[iMCompAbs_]);

                        // if (iMCompAbs_ == 464)
                        //	int debug = 0;

                        // if (!isfinite(fLabel[iMCompAbs_]))
                        //	int debug = 0;

                        SortedListUpdate<SortIndexListEntry<float>, float>(Q, binSize, pQEntry_);
                    }
                }

                pEdgePtr = pEdgePtr->pNext;
            }

            pQEntry = pQEntry->pNext;
        }
    }

    printf("100%%\n");

    delete[] Q.Element;
    delete[] QEntry;
    delete[] w;
}

void VNClassifier::ComponentAssociationMST()
{
    // Read representatives from file.

    FILE *fpRepresentatives = fopen(representativesFileName, "r");

    Array<Pair<int, int>> representatives;

    representatives.n = 0;

    int iRefModel, iRefComp, readCount;

    while (!feof(fpRepresentatives))
    {
        // readCount = fscanf(fpRepresentatives, "%d\t%d\n", &iRefModel, &iRefComp);
        readCount = fscanf(fpRepresentatives, "%d\n", &iRefModel);

        // if (readCount == 2)
        if (readCount == 1)
            representatives.n++;
        else
            break;
    }

    fclose(fpRepresentatives);

    representatives.Element = new Pair<int, int>[representatives.n];

    fpRepresentatives = fopen(representativesFileName, "r");

    int iRepresentative = 0;

    while (!feof(fpRepresentatives))
    {
        // readCount = fscanf(fpRepresentatives, "%d\t%d\n", &iRefModel, &iRefComp);
        readCount = fscanf(fpRepresentatives, "%d\n", &iRefModel);

        // if (readCount == 2)
        if (readCount == 1)
        {
            representatives.Element[iRepresentative].a = iRefModel;
            // representatives.Element[iRepresentative].b = iRefComp;
            iRepresentative++;
        }
        else
            break;
    }

    fclose(fpRepresentatives);

    // Create component MST.

    int nCompsTotal = nMCompsTotalOU[0] + nMCompsTotalOU[1];

    int *label = new int[nCompsTotal];

    Array<GRAPH::MSTreeEdge> MSTInEdges;
    MSTInEdges.Element = new GRAPH::MSTreeEdge[componentAssociationGraph.EdgeArray.n];
    MSTInEdges.n = componentAssociationGraph.EdgeArray.n;

    int iEdge;
    VN_::CAGEdge *pEdge;
    GRAPH::MSTreeEdge *pMSTEdge;

    for (iEdge = 0; iEdge < componentAssociationGraph.EdgeArray.n; iEdge++)
    {
        pEdge = componentAssociationGraph.EdgeArray.Element + iEdge;
        pMSTEdge = MSTInEdges.Element + iEdge;

        pMSTEdge->iVertex[0] = pEdge->iVertex[0];
        pMSTEdge->iVertex[1] = pEdge->iVertex[1];
        pMSTEdge->cost = pEdge->cost;
    }

    Array<GRAPH::MSTreeEdge> MSTEdges;
    MSTEdges.Element = new GRAPH::MSTreeEdge[nCompsTotal - 1];

    char MSTFileNameExtension[] = "XX.mst";
    char CASFileNameExtension[] = "XX.cas";

    MSTree MST;
    int nRepresentatives;

    // for (nRepresentatives = 1; nRepresentatives <= representatives.n; nRepresentatives++)
    for (nRepresentatives = representatives.n; nRepresentatives <= representatives.n; nRepresentatives++)
    {
        memset(label, 0xff, nCompsTotal * sizeof(int));

        int iMCompAbs, iMComp;
        VNInstance *pModel;

        for (iRepresentative = 0; iRepresentative < nRepresentatives; iRepresentative++)
        {
            iRefModel = representatives.Element[iRepresentative].a;
            // iRefComp = representatives.Element[iRepresentative].b;

            pModel = modelVNArray.Element[iRefModel];

            iMCompAbs = firstComponentAbsIdx.Element[iRefModel];

            for (iMComp = 0; iMComp < pModel->nComponents; iMComp++, iMCompAbs++)
                // label[iMCompAbs] = (iMComp == iRefComp ? 0 : 1);
                label[iMCompAbs] = iMCompAbs;
        }

        MST.Create(MSTInEdges, &MSTEdges, 0.0f, label);

        sprintf(MSTFileNameExtension, "%02d.mst", nRepresentatives);

        char *MSTFileName = RVLCreateFileName(modelDataBase, ".dat", -1, MSTFileNameExtension);

        FILE *fpMST = fopen(MSTFileName, "w");

        int i;

        for (i = 0; i < MSTEdges.n; i++)
            fprintf(fpMST, "%d\t%d\t%f\n", MSTEdges.Element[i].iVertex[0], MSTEdges.Element[i].iVertex[1], MSTEdges.Element[i].cost);

        fclose(fpMST);

        // FILE *fpLabels = fopen((std::string(resultsFolder) + "\\labels.txt").data(), "w");

        // for (i = 0; i < nCompsTotal; i++)
        //	fprintf(fpLabels, "%d\n", (label[i] == 0 ? 1 : (label[i] == 1 ? -1 : 0)));

        // fclose(fpLabels);

        sprintf(CASFileNameExtension, "%02d.cas", nRepresentatives);

        char *associationFileName = RVLCreateFileName(modelDataBase, ".dat", -1, CASFileNameExtension);

        FILE *fpAssociatedComp = fopen(associationFileName, "w");

        delete[] associationFileName;

        for (i = 0; i < nCompsTotal; i++)
            fprintf(fpAssociatedComp, "%d\n", label[i]);

        fclose(fpAssociatedComp);
    }

    delete[] MSTInEdges.Element;
    delete[] MSTEdges.Element;
    delete[] label;
    delete[] representatives.Element;
}

void VNClassifier::DetectSupportingSurfaces(Mesh *pMesh)
{
    planeDetector.pMesh = pMesh;

    planeDetector.DetectPlanes();

    // Surfels belonging to planar surfaces are excluded from detection of convex and concave surfaces.

    int maxPlaneSize = 0;

    int i;
    int iSegment;
    Surfel *pSurfel;
    PSGM_::Cluster *pPlane;

    for (iSegment = 0; iSegment < planeDetector.clusters.n; iSegment++)
    {
        pPlane = planeDetector.clusters.Element[iSegment];

        for (i = 0; i < pPlane->iSurfelArray.n; i++)
        {
            pSurfel = pSurfels->NodeArray.Element + pPlane->iSurfelArray.Element[i];

            pSurfel->flags |= RVLSURFEL_FLAG_GND;
        }

        if (pPlane->size > maxPlaneSize)
        {
            maxPlaneSize = pPlane->size;

            RVLCOPY3VECTOR(pPlane->N, NGnd);

            dGnd = pPlane->dPlane;
        }
    }
}

void VN_::_3DNetDatabaseClasses(VNClassifier *pClassifier)
{
    pClassifier->classArray.n = 10;

    pClassifier->classArray.Element = new RECOG::ClassData[pClassifier->classArray.n];

    RECOG::ClassData *pClass;

    // class apple
    pClass = pClassifier->classArray.Element;
    pClass->iMetaModel = RVLVN_METAMODEL_CONVEX;
    pClass->iFirstInstance = 0;
    pClass->nInstances = 12;
    pClass->iRefInstance = pClass->iFirstInstance; // by default

    pClass->VNGraspNormals_.n = 1;
    pClass->VNGraspNormals_.Element = new RECOG::VNGraspNormals[pClass->VNGraspNormals_.n];
    pClass->VNGraspNormals_.Element[0].n1 = 11; // parallel planes for gripping
    pClass->VNGraspNormals_.Element[0].n2 = 44; // parallel planes for gripping
    pClass->VNGraspNormals_.Element[0].n3 = 15; // adjacent planes
    pClass->VNGraspNormals_.Element[0].n4 = 18; // adjacent planes
    pClass->VNGraspNormals_.Element[0].n5 = 0;

    // class banana
    pClass = pClassifier->classArray.Element + 1;
    // pClass->iMetaModel = RVLVN_METAMODEL_BANANA;
    pClass->iMetaModel = RVLVN_METAMODEL_TORUS;
    pClass->iFirstInstance = 12;
    pClass->nInstances = 6;
    pClass->iRefInstance = 17;

    pClass->VNGraspNormals_.n = 1;
    pClass->VNGraspNormals_.Element = new RECOG::VNGraspNormals[pClass->VNGraspNormals_.n];
    pClass->VNGraspNormals_.Element[0].n1 = 55;
    pClass->VNGraspNormals_.Element[0].n2 = 72;
    pClass->VNGraspNormals_.Element[0].n3 = 50;
    pClass->VNGraspNormals_.Element[0].n4 = 57;
    pClass->VNGraspNormals_.Element[0].n5 = 0;

    // class bottle
    pClass = pClassifier->classArray.Element + 2;
    pClass->iMetaModel = RVLVN_METAMODEL_BOTTLE;
    pClass->iFirstInstance = 18;
    pClass->nInstances = 74;
    pClass->iRefInstance = pClass->iFirstInstance;

    pClass->VNGraspNormals_.n = 1;
    pClass->VNGraspNormals_.Element = new RECOG::VNGraspNormals[pClass->VNGraspNormals_.n];
    pClass->VNGraspNormals_.Element[0].n1 = 87;
    pClass->VNGraspNormals_.Element[0].n2 = 72;
    pClass->VNGraspNormals_.Element[0].n3 = 89;
    pClass->VNGraspNormals_.Element[0].n4 = 91;
    pClass->VNGraspNormals_.Element[0].n5 = 61;

    // class bowl
    pClass = pClassifier->classArray.Element + 3;
    pClass->iMetaModel = RVLVN_METAMODEL_BOWL;
    pClass->iFirstInstance = 92;
    pClass->nInstances = 31;
    pClass->iRefInstance = pClass->iFirstInstance;

    pClass->VNGraspNormals_.n = 1;
    pClass->VNGraspNormals_.Element = new RECOG::VNGraspNormals[pClass->VNGraspNormals_.n];
    // pClass->VNGraspNormals_.Element[0].n1 = 0;
    // pClass->VNGraspNormals_.Element[0].n2 = 0;
    // pClass->VNGraspNormals_.Element[0].n3 = 0;
    // pClass->VNGraspNormals_.Element[0].n4 = 0;
    // pClass->VNGraspNormals_.Element[0].n5 = 0;

    // class car
    pClass = pClassifier->classArray.Element + 4;
    pClass->iMetaModel = RVLVN_METAMODEL_CONVEX;
    pClass->iFirstInstance = 123;
    pClass->nInstances = 72;
    // pClass->iRefInstance = 172;
    pClass->iRefInstance = 123;

    pClass->VNGraspNormals_.n = 1;
    pClass->VNGraspNormals_.Element = new RECOG::VNGraspNormals[pClass->VNGraspNormals_.n];
    pClass->VNGraspNormals_.Element[0].n1 = 55;
    pClass->VNGraspNormals_.Element[0].n2 = 22;
    pClass->VNGraspNormals_.Element[0].n3 = 58;
    pClass->VNGraspNormals_.Element[0].n4 = 61;
    pClass->VNGraspNormals_.Element[0].n5 = 0;

    // class donut
    pClass = pClassifier->classArray.Element + 5;
    pClass->iMetaModel = RVLVN_METAMODEL_TORUS;
    pClass->iFirstInstance = 195;
    pClass->nInstances = 10;
    pClass->iRefInstance = pClass->iFirstInstance;

    pClass->VNGraspNormals_.n = 1;
    pClass->VNGraspNormals_.Element = new RECOG::VNGraspNormals[pClass->VNGraspNormals_.n];
    // pClass->VNGraspNormals_.Element[0].n1 = 55;
    // pClass->VNGraspNormals_.Element[0].n2 = 22;
    // pClass->VNGraspNormals_.Element[0].n3 = 58;
    // pClass->VNGraspNormals_.Element[0].n4 = 61;
    // pClass->VNGraspNormals_.Element[0].n5 = 0;

    // class hammer
    pClass = pClassifier->classArray.Element + 6;
    pClass->iMetaModel = RVLVN_METAMODEL_BOTTLE;
    pClass->iFirstInstance = 205;
    pClass->nInstances = 36;
    pClass->iRefInstance = pClass->iFirstInstance;

    pClass->VNGraspNormals_.n = 1;
    pClass->VNGraspNormals_.Element = new RECOG::VNGraspNormals[pClass->VNGraspNormals_.n];
    pClass->VNGraspNormals_.Element[0].n1 = 99;
    pClass->VNGraspNormals_.Element[0].n2 = 85;
    pClass->VNGraspNormals_.Element[0].n3 = 89;
    pClass->VNGraspNormals_.Element[0].n4 = 26;
    pClass->VNGraspNormals_.Element[0].n5 = 92;

    // class mug
    pClass = pClassifier->classArray.Element + 7;
    pClass->iMetaModel = RVLVN_METAMODEL_MUG;
    pClass->iFirstInstance = 241;
    pClass->nInstances = 75;
    pClass->iRefInstance = 245;

    pClass->VNGraspNormals_.n = 1;
    pClass->VNGraspNormals_.Element = new RECOG::VNGraspNormals[pClass->VNGraspNormals_.n];
    pClass->VNGraspNormals_.Element[0].n1 = 11;
    pClass->VNGraspNormals_.Element[0].n2 = 92;
    pClass->VNGraspNormals_.Element[0].n3 = 15;
    pClass->VNGraspNormals_.Element[0].n4 = 18;
    pClass->VNGraspNormals_.Element[0].n5 = 0;

    // class tetra pak
    pClass = pClassifier->classArray.Element + 8;
    pClass->iMetaModel = RVLVN_METAMODEL_CONVEX;
    pClass->iFirstInstance = 316;
    pClass->nInstances = 26;
    pClass->iRefInstance = pClass->iFirstInstance;

    pClass->VNGraspNormals_.n = 1;
    pClass->VNGraspNormals_.Element = new RECOG::VNGraspNormals[pClass->VNGraspNormals_.n];
    // pClass->VNGraspNormals_.Element[0].n1 = 55;
    // pClass->VNGraspNormals_.Element[0].n2 = 22;
    // pClass->VNGraspNormals_.Element[0].n3 = 58;
    // pClass->VNGraspNormals_.Element[0].n4 = 61;
    // pClass->VNGraspNormals_.Element[0].n5 = 0;

    // class toilet paper
    pClass = pClassifier->classArray.Element + 9;
    // pClass->iMetaModel = RVLVN_METAMODEL_PIPE;
    pClass->iMetaModel = RVLVN_METAMODEL_BOWL;
    pClass->iFirstInstance = 342;
    pClass->nInstances = 9;
    pClass->iRefInstance = pClass->iFirstInstance;

    pClass->VNGraspNormals_.n = 1;
    pClass->VNGraspNormals_.Element = new RECOG::VNGraspNormals[pClass->VNGraspNormals_.n];
    // pClass->VNGraspNormals_.Element[0].n1 = 55;
    // pClass->VNGraspNormals_.Element[0].n2 = 22;
    // pClass->VNGraspNormals_.Element[0].n3 = 58;
    // pClass->VNGraspNormals_.Element[0].n4 = 61;
    // pClass->VNGraspNormals_.Element[0].n5 = 0;

    // common values for all classes

    int iClass;

    for (iClass = 0; iClass < pClassifier->classArray.n; iClass++)
    {
        pClass = pClassifier->classArray.Element + iClass;

        pClass->M.Element = NULL;
        pClass->qMax = NULL;
        pClass->qMin = NULL;
        pClass->A = NULL;
        pClass->b = NULL;
        pClass->q0 = NULL;
        pClass->valid = NULL;
        pClass->nq0 = 1;
        pClass->bConvex = pClass->bConcave = false;
    }

    // create VN model for every class.
    if (pClassifier->bTrainingStructure)
    {
        pClassifier->models = pClassifier->metaModels;
    }
    else
    {
        int i;
        for (int iClass = 0; iClass < pClassifier->classArray.n; iClass++)
        {
            pClass = pClassifier->classArray.Element + iClass;
            i = 0;

            std::string validFeaturesFileName = std::string(pClassifier->validFeaturesFileName) + std::to_string(iClass) + ".txt";
            std::ifstream infile(validFeaturesFileName);
            std::string unused;
            while (std::getline(infile, unused))
                ++i;
            infile.close();
            infile.open(validFeaturesFileName);
            pClass->valid = new int[i];
            for (int line = 0; line < i; line++)
            {
                infile >> pClass->valid[line];
            }
            infile.close();
            VN *pModel = new VN;
            VN_::Create(pClassifier->metaModels[pClass->iMetaModel], pClass->valid, pClassifier->alignment.convexTemplate66, pClassifier->pMem0, pModel);
            pClassifier->models.push_back(pModel);
            pClass->iMetaModel = iClass;

            char metaModelFileName[] = "metamodel0.dat";

            FILE *fp;

            sprintf(metaModelFileName + strlen(metaModelFileName) - 5, "%1d.dat", iClass);
            fp = fopen(metaModelFileName, "w");

            pModel->SaveFeatures(fp);

            fclose(fp);
        }
    }

    // special property of class mug

    pClass = pClassifier->classArray.Element + 7;

    pClass->nq0 = 2;
    // pClass->q0 = new float[pClass->nq0 * ];
}

void VN_::OSDDatabaseClasses(VNClassifier *pClassifier)
{
    pClassifier->classArray.n = 4;

    pClassifier->classArray.Element = new RECOG::ClassData[pClassifier->classArray.n];

    RECOG::ClassData *pClass, *pMug;
    int i, j;
    float z;

    // class mug

    pClass = pClassifier->classArray.Element + 0;
    pClass->iMetaModel = RVLVN_METAMODEL_CONVEX;
    // pClass->iFirstInstance = 0;
    // pClass->nInstances = 12;
    // pClass->iRefInstance = pClass->iFirstInstance; //by default
    pClass->bConvex = false;
    pClass->bAxisAlignment = false;
#ifdef RVLVN_CLASSIFY_CTI_VERSION1B
    pClass->faces.n = 6;
    pClass->faces.Element = new int[pClass->faces.n];
    for (j = 0; j < 6; j++)
        pClass->faces.Element[j] = 11 * j;
#else
    pClass->faces.n = 18;
    pClass->faces.Element = new int[pClass->faces.n];
    pClass->faces.Element[0] = 0;
    pClass->faces.Element[1] = 33;
    j = 2;
    for (i = 0; i < pClassifier->sampledUnitSphere.h; i++)
    {
        z = pClassifier->sampledUnitSphere.Element[3 * i + 2];

        if (RVLABS(z) < 1e-6)
            pClass->faces.Element[j++] = i;
    }
#endif
    pMug = pClass;
    pClass->iHypothesisBlocks.Element = pClass->iHypotheisisBlocksMem;
    pClass->iHypothesisBlocks.h = 6;
    pClass->iHypothesisBlocks.w = 1;
    for (i = 0; i < 6; i++)
        pClass->iHypothesisBlocks.Element[i] = i;

    // class bowl

    pClass = pClassifier->classArray.Element + 1;
    pClass->iMetaModel = RVLVN_METAMODEL_CONVEX;
    pClass->bConvex = false;
    pClass->bAxisAlignment = true;
#ifdef RVLVN_CLASSIFY_CTI_VERSION1B
    pClass->faces.n = 6;
    pClass->faces.Element = new int[pClass->faces.n];
    for (j = 0; j < 6; j++)
        pClass->faces.Element[j] = 11 * j;
#else
    pClass->faces.n = 2;
    pClass->faces.Element = new int[pClass->faces.n];
    pClass->faces.Element[0] = 0;
    pClass->faces.Element[1] = 33;
#endif
    pClass->iHypothesisBlocks.Element = pClass->iHypotheisisBlocksMem;
    pClass->iHypothesisBlocks.h = 2;
    pClass->iHypothesisBlocks.w = 3;
    for (i = 0; i < 6; i++)
        pClass->iHypothesisBlocks.Element[i] = i;

    // class box

    pClass = pClassifier->classArray.Element + 2;
    pClass->iMetaModel = RVLVN_METAMODEL_CONVEX;
    pClass->bConvex = true;
    pClass->bAxisAlignment = false;
    pClass->faces.n = 6;
    pClass->faces.Element = new int[pClass->faces.n];
    for (j = 0; j < 6; j++)
        pClass->faces.Element[j] = 11 * j;
    pClass->iHypothesisBlocks.Element = pClass->iHypotheisisBlocksMem;
    pClass->iHypothesisBlocks.h = 1;
    pClass->iHypothesisBlocks.w = 6;
    for (i = 0; i < 6; i++)
        pClass->iHypothesisBlocks.Element[i] = i;

    // class cylinder

    pClass = pClassifier->classArray.Element + 3;
    pClass->iMetaModel = RVLVN_METAMODEL_CONVEX;
    pClass->bConvex = false;
    pClass->bAxisAlignment = false;
#ifdef RVLVN_CLASSIFY_CTI_VERSION1B
    pClass->faces.n = 6;
    pClass->faces.Element = new int[pClass->faces.n];
    for (j = 0; j < 6; j++)
        pClass->faces.Element[j] = 11 * j;
#else
    pClass->faces.n = pMug->faces.n;
    pClass->faces.Element = new int[pClass->faces.n];
    memcpy(pClass->faces.Element, pMug->faces.Element, pClass->faces.n * sizeof(int));
#endif
    pClass->iHypothesisBlocks.Element = pClass->iHypotheisisBlocksMem;
    pClass->iHypothesisBlocks.h = 3;
    pClass->iHypothesisBlocks.w = 2;
    for (i = 0; i < 3; i++)
    {
        pClass->iHypothesisBlocks.Element[pClass->iHypothesisBlocks.w * i] = i;
        pClass->iHypothesisBlocks.Element[pClass->iHypothesisBlocks.w * i + 1] = i + 3;
    }

    // common values for all classes

    int iClass;

    for (iClass = 0; iClass < pClassifier->classArray.n; iClass++)
    {
        pClass = pClassifier->classArray.Element + iClass;

        pClass->M.Element = NULL;
        pClass->qMax = NULL;
        pClass->qMin = NULL;
        pClass->q0 = NULL;
        pClass->nq0 = 1;
        pClass->VNGraspNormals_.Element = NULL;
        pClass->valid = NULL;
        pClass->bConcave = false;
    }

    // Copy metamodels to models.

    pClassifier->models = pClassifier->metaModels;
}

void VN_::ShapeNetDatabaseClasses(VNClassifier *pClassifier)
{
    pClassifier->classArray.n = 2;

    pClassifier->classArray.Element = new RECOG::ClassData[pClassifier->classArray.n];

    RECOG::ClassData *pClass;
    int i, j;

    // class convex

    pClass = pClassifier->classArray.Element + 0;
    pClass->iMetaModel = RVLVN_METAMODEL_CONVEX;
    pClass->bConvex = true;
    pClass->bConcave = false;
    pClass->bAxisAlignment = false;
    pClass->faces.n = 6;
    pClass->faces.Element = new int[pClass->faces.n];
    for (j = 0; j < 6; j++)
        pClass->faces.Element[j] = 11 * j;

    // class concave

    pClass = pClassifier->classArray.Element + 1;
    pClass->iMetaModel = RVLVN_METAMODEL_CONVEX;
    pClass->bConvex = false;
    pClass->bConcave = true;
    pClass->bAxisAlignment = false;
    pClass->faces.n = 6;
    pClass->faces.Element = new int[pClass->faces.n];
    for (j = 0; j < 6; j++)
        pClass->faces.Element[j] = 11 * j;

    // common values for all classes

    int iClass;

    for (iClass = 0; iClass < pClassifier->classArray.n; iClass++)
    {
        pClass = pClassifier->classArray.Element + iClass;

        pClass->M.Element = NULL;
        pClass->qMax = NULL;
        pClass->qMin = NULL;
        pClass->q0 = NULL;
        pClass->nq0 = 1;
        pClass->VNGraspNormals_.Element = NULL;
        pClass->valid = NULL;
    }

    // Copy metamodels to models.

    pClassifier->models = pClassifier->metaModels;
}

bool VN_::keyPressUserFunction(
    Mesh *pMesh,
    SurfelGraph *pSurfels,
    std::string &key,
    void *vpData)
{
    VN_::VisualizationData *pData = (VN_::VisualizationData *)vpData;

    VNClassifier *pClassifier = pData->pClassifier;
    Visualizer *pVisualizer = pData->pVisualizer;

    if (key == "c")
    {
        pData->pVNSceneInstance->PaintComponent(pData->pVisualizer, pData->pMesh, pData->pSurfels, pData->iSelectedComponent, pData->clusterColor + 3 * pData->iSelectedComponent);
        pData->iSelectedComponent = -1;
        printf("No component selected.\n");
    }
    if (key == "h")
    {
        int i;

        for (i = 0; i < pData->topHypothesisActors.n; i++)
            pVisualizer->renderer->RemoveViewProp(pData->topHypothesisActors.Element[i]);

        if (pData->bHypothesisSelected)
        {
            pVisualizer->renderer->RemoveViewProp(pData->selectedHypothesisActor[0]);
            pVisualizer->renderer->RemoveViewProp(pData->selectedHypothesisActor[1]);
            pVisualizer->renderer->RemoveViewProp(pData->selectedHypothesisActor[2]);
        }

        printf("Enter supersegment ID and hypothesis rank: ");

        int iSuperSegment, rank;

        scanf("%d %d", &iSuperSegment, &rank);

        VN_::Hypothesis2 *pHypothesis;

        if (iSuperSegment >= 0)
            pHypothesis = pClassifier->sortedSuperSegmentHypotheses[iSuperSegment].Element[rank].ptr;
        else
        {
            pHypothesis = pClassifier->hypothesisList.pFirst;

            while (pHypothesis)
            {
                if (pHypothesis->idx == rank)
                    break;

                pHypothesis = pHypothesis->pNext;
            }
        }

        if (pHypothesis)
        {
            int displayType = 0;

            if (displayType == 0)
            {
                double cyan[3] = {0.0f, 1.0f, 1.0f};

                vtkSmartPointer<vtkActor> actor = pClassifier->pShapeInstanceDetection->AddModelToVisualizer(pVisualizer, pClassifier->pShapeInstanceDetection->vtkModelDB[pHypothesis->iModel],
                                                                                                             pHypothesis->R, pHypothesis->P, cyan, 2.0f);
            }
            else if (displayType == 1)
            {
                PSGM_::MatchInstance match;

                int nVisiblePts, nTransparentPts;

                Array<OrientedPoint> model = pClassifier->pShapeInstanceDetection->sampledModels.Element[pHypothesis->iModel];

                Array<int> ptIdxSet[3];

                for (i = 0; i < 3; i++)
                    ptIdxSet[i].Element = new int[model.n];

                VN_::SurfelMask surfelMask;

                pClassifier->CreateSurfelMask(pMesh, surfelMask);

                pClassifier->CreateSurfelMask(pHypothesis->segments, surfelMask);

                pClassifier->pShapeInstanceDetection->surfelMask = surfelMask.mask;

                float score;

                if (pClassifier->hypothesisEvaluationLevel3Method == 1)
                {
                    score = pClassifier->pShapeInstanceDetection->HypothesisEvaluationIP(pClassifier->pShapeInstanceDetection->sampledModels.Element[pHypothesis->iModel], pHypothesis->R, pHypothesis->P, 0.03f,
                                                                                         nVisiblePts, nTransparentPts, ptIdxSet, ptIdxSet + 1, ptIdxSet + 2);

                    uchar color[3][3] = {{0, 255, 0}, {255, 0, 0}, {0, 0, 255}};

                    float *R = pHypothesis->R;
                    float *t = pHypothesis->P;

                    Array<Point> ptSet;
                    int j;
                    float *PSrc, *PTgt;

                    for (i = 0; i < 3; i++)
                    {
                        ptSet.Element = new Point[ptIdxSet[i].n];
                        ptSet.n = ptIdxSet[i].n;

                        for (j = 0; j < ptIdxSet[i].n; j++)
                        {
                            PSrc = model.Element[ptIdxSet[i].Element[j]].P;

                            PTgt = ptSet.Element[j].P;

                            RVLTRANSF3(PSrc, R, t, PTgt);
                        }

                        pData->selectedHypothesisActor[i] = pVisualizer->DisplayPointSet<float, Point>(ptSet, color[i], 5.0f);

                        delete[] ptSet.Element;
                        delete[] ptIdxSet[i].Element;
                    }
                }
                else
                {
                    int *SMCorrespondence = new int[pClassifier->pShapeInstanceDetection->ZBuffer.w * pClassifier->pShapeInstanceDetection->ZBuffer.h];

                    score = pClassifier->pShapeInstanceDetection->HypothesisEvaluationIP2(pClassifier->pShapeInstanceDetection->modelPCs[pHypothesis->iModel], pHypothesis->R, pHypothesis->P, 1.0f, 0.01f,
                                                                                          nTransparentPts, SMCorrespondence);

                    pClassifier->pShapeInstanceDetection->DisplayHypothesisEvaluationIP2(pVisualizer, SMCorrespondence, nTransparentPts, pData->selectedHypothesisActor);

                    nVisiblePts = pClassifier->pShapeInstanceDetection->ZBufferActivePtArray.n;
                }

                printf("nVisiblePts: %d score: %f nTransparentPts: %d iModel: %d\n", nVisiblePts, score, nTransparentPts, pHypothesis->iModel);

                // double green[3] = { 0, 1.0, 0 };

                // pData->selectedHypothesisActor = pClassifier->pShapeInstanceDetection->AddModelToVisualizer(pVisualizer, pClassifier->pShapeInstanceDetection->vtkModelDB[pHypothesis->iModel],
                //	pHypothesis->R, pHypothesis->P, green);

                pData->bHypothesisSelected = true;

                pClassifier->DisplaySurfelMask(surfelMask);
            }
        }
    }
    else if (key == "i")
    {
        int iSelectedCluster;

        if (pClassifier->SClusters.n > 1)
        {
            printf("Enter cluster identifier (0-%d): ", pClassifier->SClusters.n - 1);

            scanf("%d", &iSelectedCluster);
        }
        else
            iSelectedCluster = 0;

        if (iSelectedCluster >= 0 && iSelectedCluster < pClassifier->SClusters.n)
        {
            pClassifier->DisplaySelectedCluster(iSelectedCluster);

            pData->iSelectedCluster = iSelectedCluster;

            if (pData->bVisualizeZAxes)
            {
                if (pData->ZAxesActor)
                    pData->pVisualizer->renderer->RemoveViewProp(pData->ZAxesActor);

                pData->ZAxesActor = DisplayZAxes(pData->pVisualizer, pData->ZAxes + iSelectedCluster, 0.05f * pData->meshSize);
            }

            return true;
        }
        else
            return false;
    }
    else if (key == "s")
    {
        printf("Enter super segment number (0-%d): ", pClassifier->superSegments.n);

        int iSelectedSuperSegment;

        scanf("%d", &iSelectedSuperSegment);

        if (iSelectedSuperSegment >= 0 && iSelectedSuperSegment < pClassifier->superSegments.n)
        {
            pClassifier->DisplaySelectedSuperSegment(iSelectedSuperSegment);

            pData->iSelectedSuperSegment = iSelectedSuperSegment;

            return true;
        }
        else
            return false;
    }
    // Visualize Part Association evaluation
    if (key == "p")
    {
        printf("Enter label number (1 to %d):", pClassifier->maxGTPartLabel);

        int iSelectedLabel;

        scanf("%d", &iSelectedLabel);

        if (iSelectedLabel > 0 && iSelectedLabel <= pClassifier->maxGTPartLabel)
        {
            pData->pVNSceneInstance->VisualizePartAssociation(pMesh, pClassifier, 0, pData->componentAssociation, pData->cellAssociation, iSelectedLabel, 1);
        }
    }
    // Assign labels to unassigned points and Visualize Part Association
    if (key == "a")
    {
        if (!pClassifier->bLabelsAssignedToUnassignedPoints)
        {
            pClassifier->bLabelsAssignedToUnassignedPoints = true;
            pData->pVNSceneInstance->VisualizePartAssociation(pMesh, pClassifier, 0, pData->componentAssociation, pData->cellAssociation, -1, 1);
        }
        else
        {
            pClassifier->bLabelsAssignedToUnassignedPoints = false;
            // pData->pVNSceneInstance->VisualizePartAssociation(pMesh, pClassifier, 0, pData->association, -1, -1);
        }
    }

    if (key == "g")
    {
        pData->pVNSceneInstance->VisualizeGTLabels(pMesh, pClassifier);
    }

    if (key == "o")
    {
        pData->pVNSceneInstance->VisualizeSparseGTLabels(pMesh, pClassifier);
    }

    if (key == "k")
    {
        if (pData->iSelectedSuperSegment < 0)
        {
            for (int iSuperSegment = 0; iSuperSegment < pClassifier->superSegments.n; iSuperSegment++)
            {
                for (int iCluster = 0; iCluster < pClassifier->superSegments.Element[iSuperSegment]->segments.n; iCluster++)
                {
                    if (pData->iSelectedCluster == pClassifier->superSegments.Element[iSuperSegment]->segments.Element[iCluster])
                    {
                        pData->iSelectedSuperSegment = iSuperSegment;

                        break;
                    }
                }
            }
        }

        pData->iSelectedComponent = pData->iSelectedSuperSegment;
    }

    return false;
}

bool VN_::mouseRButtonDownUserFunction(
    Mesh *pMesh,
    SurfelGraph *pSurfels,
    int iSelectedPt,
    int iSelectedSurfel,
    void *vpData)
{
    VN_::VisualizationData *pData = (VN_::VisualizationData *)vpData;

    if (!pData->bClusters)
        return false;

    VNClassifier *pClassifier = pData->pClassifier;

    if (pData->bVisualizeVNInstance)
    {
        int iCell;
        SURFEL::Cell *pCell;
        QLIST::Index *pSurfelIdx;

        for (iCell = 0; iCell < pData->pVNSceneInstance->surfelCells.n; iCell++)
        {
            pCell = pData->pVNSceneInstance->surfelCells.Element + iCell;

            pSurfelIdx = pCell->surfelList.pFirst;

            while (pSurfelIdx)
            {
                if (pSurfelIdx->Idx == iSelectedSurfel)
                    break;

                pSurfelIdx = pSurfelIdx->pNext;
            }

            if (pSurfelIdx)
                break;
        }

        if (iCell < pData->pVNSceneInstance->surfelCells.n)
        {
            Array<int> objects;

            objects.Element = new int[pData->pVNSceneInstance->nComponents];
            objects.n = 0;

            QLIST::Index *pObjectIdx = pCell->objectList.pFirst;

            while (pObjectIdx)
            {
                objects.Element[objects.n++] = pObjectIdx->Idx;

                pObjectIdx = pObjectIdx->pNext;
            }

            int iSelectedComponent;

            if (objects.n > 1)
            {
                printf("Select component (");

                int i;

                for (i = 0; i < objects.n; i++)
                {
                    printf("%d", objects.Element[i]);

                    if (i < objects.n - 1)
                        printf(", ");
                }

                printf("): ");

                scanf("%d", &(iSelectedComponent));
            }
            else
                iSelectedComponent = objects.Element[0];

            if (iSelectedComponent >= 0 && iSelectedComponent < pData->pVNSceneInstance->nComponents)
            {
                pData->pVNSceneInstance->PaintComponent(pData->pVisualizer, pData->pMesh, pData->pSurfels, pData->iSelectedComponent, pData->clusterColor + 3 * pData->iSelectedComponent);
                pData->iSelectedComponent = iSelectedComponent;
                pData->pVNSceneInstance->PaintComponent(pData->pVisualizer, pData->pMesh, pData->pSurfels, pData->iSelectedComponent, pData->selectionColor);
                printf("Selected component: %d\n", iSelectedComponent);
            }

            delete[] objects.Element;
        }

        return true;
    }
    else
    {
        int iSelectedCluster = pClassifier->clusterMap[iSelectedSurfel];

        pClassifier->DisplaySelectedCluster(iSelectedCluster);

        if (iSelectedCluster >= 0)
        {
            pData->iSelectedCluster = iSelectedCluster;

            pData->iSelectedSuperSegment = -1;

            return true;
        }
        else
            return false;
    }

    // unsigned char color[3];
    // SceneCluster *pCluster;
    // PSGM_::Cluster *pCluster_;

    // if (pData->iSelectedCluster >= 0)
    //{
    //	pClassifier->ResetClusterColor(pData->iSelectedCluster);

    //	if (pSurfels->DisplayData.bVertices)
    //	{
    //		RVLSET3VECTOR(color, 255, 0, 0);

    //		pCluster = pClassifier->SClusters.Element + pData->iSelectedCluster;

    //		pCluster_ = (PSGM_::Cluster *)(pCluster->vpCluster);

    //		pSurfels->PaintVertices(&(pCluster_->iVertexArray), color);
    //	}
    //}

    //

    // if (iCluster >= 0)
    //{
    //	printf("Selected segment: %d\n", iCluster);

    //	pCluster = pClassifier->SClusters.Element + iCluster;

    //	pCluster_ = (PSGM_::Cluster *)(pCluster->vpCluster);

    //	pSurfels->PaintSurfels(pMesh, pVisualizer, pCluster_->iSurfelArray, pData->selectionColor);

    //	if (pSurfels->DisplayData.bVertices)
    //	{
    //		RVLSET3VECTOR(color, 255, 255, 0);

    //		pCluster = pClassifier->SClusters.Element + iCluster;

    //		pCluster_ = (PSGM_::Cluster *)(pCluster->vpCluster);

    //		pSurfels->PaintVertices(&(pCluster_->iVertexArray), color);
    //	}

    //	pData->iSelectedCluster = iCluster;

    //	return true;
    //}
    // else
    //	return false;
}

float VN_::ComparePosesUsingConvexHull(
    float *R,
    float *t,
    Mesh *pCH,
    float *PMem)
{
    float *P_ = (PMem ? PMem : new float[3 * pCH->iValidVertices.n]);

    float *P__ = P_;

    int i;
    float *P;

    for (i = 0; i < pCH->iValidVertices.n; i++, P__ += 3)
    {
        P = pCH->NodeArray.Element[pCH->iValidVertices.Element[i]].P;

        RVLTRANSF3(P, R, t, P__);
    }

    float E = 0.0f;

    int j;
    MESH::Face *pFace;
    float d, d_, e;

    for (j = 0; j < pCH->iVisibleFaces.n; j++)
    {
        pFace = pCH->faces.Element[pCH->iVisibleFaces.Element[j]];

        P__ = P_;

        d = RVLDOTPRODUCT3(pFace->N, P__);

        P__ += 3;

        for (i = 1; i < pCH->iValidVertices.n; i++, P__ += 3)
        {
            d_ = RVLDOTPRODUCT3(pFace->N, P__);

            if (d_ > d)
                d = d_;
        }

        e = d - pFace->d;

        E += (e * e);

        // if (e < 0.0f)
        //	e = -e;

        // if (e > E)
        //	E = e;
    }

    if (PMem == NULL)
        delete[] P_;

    return E / (float)(pCH->iVisibleFaces.n);

    // return E;
}

// NOT FINISHED - MOVED TO PSGM.cpp
// bool VNClassifier::ReferenceFrames(Mesh convexHull)
//{
//	int i, j;
//	float score;
//	float dotProduct, normi, normj;
//	Array<convexHullFacesPairs> convexHullFacesPairArr;
//
//	convexHullFacesPairArr.n = convexHull.faces.n * convexHull.faces.n / 2;
//	convexHullFacesPairArr.Element = new convexHullFacesPairs[convexHullFacesPairArr.n];
//	int br = 0;
//
//	for (i = 0; i < convexHull.faces.n; i++)
//		for (j = i+1; j < convexHull.faces.n; j++)
//		{
//			dotProduct = RVLDOTPRODUCT3(convexHull.faces.Element[i]->N, convexHull.faces.Element[j]->N);
//			if (RVLABS(dotProduct) < 0.7)
//			{
//				score = convexHull.faces.Element[i]->Area * convexHull.faces.Element[j]->Area;
//				convexHullFacesPairArr.Element[br].cost = score;
//				convexHullFacesPairArr.Element[br].faces->a = i;
//				convexHullFacesPairArr.Element[br].faces->b = j;
//				br++;
//			}
//		}
//
//	convexHullFacesPairArr.n = br;
//
//	BubbleSort<convexHullFacesPairs>(convexHullFacesPairArr, true);
//
//	int nBestFacesPairs = 50;
//	float theta;
//	Array<convexHullFacesPairs> convexHullFacesPairArr_;
//	float R[9], v[3], x[3], y[3], z[3];
//
//	convexHullFacesPairArr_.n = br;
//	convexHullFacesPairArr_.Element = new convexHullFacesPairs[convexHullFacesPairArr_.n];
//	br = 0;
//	for (i = 0; i < nBestFacesPairs; i++)
//	{
//
//		z[0] = convexHull.faces.Element[convexHullFacesPairArr.Element[i].faces->a]->N[0];
//		z[1] = convexHull.faces.Element[convexHullFacesPairArr.Element[i].faces->a]->N[1];
//		z[2] = convexHull.faces.Element[convexHullFacesPairArr.Element[i].faces->a]->N[2];
//
//
//		RVLCROSSPRODUCT3(z, convexHull.faces.Element[convexHullFacesPairArr.Element[i].faces->b]->N, x);
//		RVLCROSSPRODUCT3(z, x, y);
//
//		R[0] = x[0];
//		R[3] = x[1];
//		R[6] = x[2];
//
//		R[1] = y[0];
//		R[4] = y[1];
//		R[7] = y[2];
//
//		R[2] = z[0];
//		R[5] = z[1];
//		R[9] = z[2];
//
//		GetAngleAxis(R, v, theta);
//
//		if (theta > 0.35)
//		{
//			convexHullFacesPairArr_.Element[br] = convexHullFacesPairArr.Element[i];
//			br++;
//		}
//
//	}
//
// }
