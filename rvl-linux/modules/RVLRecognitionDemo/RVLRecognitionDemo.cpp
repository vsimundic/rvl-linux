// RVLRecognitionDemo.cpp : Defines the entry point for the console application.
//

#include "RVLCore2.h"
#ifndef RVLLINUX
#define RVLRECOGNITION_DEMO_RVLTG
#include <Windows.h>
#endif
#include <ctime>
#include <fstream>
#include <chrono>
// #include "stdafx.h"
#include <vtkAutoInit.h>
// VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
#include "RVLVTK.h"
#include <vtkTriangle.h> // Remove after completion of MarchingCubes.
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
#include "RVLRecognitionCommon.h"
#include "RVLRecognition.h"
#include "RFRecognition.h"
#include "RVLMeshNoiser.h"
#ifdef RVLRECOGNITION_DEMO_RVLTG
#include "TGraph.h"
#endif
#include "PSGMCommon.h"
#include "CTISet.h"
#include "VertexGraph.h"
#include "TG.h"
#include "TGSet.h"
#include "PSGM.h"
#ifdef RVLPLC
#include <pcl/common/common.h>
#include <pcl/registration/registration.h>
#include <pcl/PolygonMesh.h>
#endif
#ifdef RVLOPENNI
#include <pcl/io/openni2_grabber.h>
#include <pcl/io/openni2/openni2_metadata_wrapper.h>
#endif
#ifdef RVLPCL
#include "PCLTools.h"
#include "RGBDCamera.h"
#include "PCLMeshBuilder.h"
#endif
#include "NeighborhoodTool.h"
#ifdef RVLPCL
#include "PCLNeighborhoodTool.h"
#endif
#include "MarchingCubes.h"
#include "VN.h"
#include "VNClass.h"
#include "VNInstance.h"
#include "VNClassifier.h"
#include "Gripper.h"
#include "RVLBuffer.h"
#include "RVLPtrChain.h"
#include "RVLMPtrChain.h"
#include "Rect.h"
#include "RVLEDT.h"
#include "DDDetector.h"
#include "BranchMatcher.h"
#ifdef RVL_ASTRA
#include "astraUtils.h"
#endif
#include "RVLAstra.h"
#include "ObjectDetector.h"
// #define RVLPSGM_CUDA_ICP
#ifdef RVLPSGM_CUDA_ICP
#include "ICPCUDAv1.h"
#endif
// #include "ICPCUDAv2.h"
// #include <sophus/se3.hpp>
#include "slic.h"

bool bVerbose = false;

// VIDOVIC
// #define RVL_COORDINATE_SYSTEM_NOISE_STABILITY_TEST
// #define RVL_COORDINATE_SYSTEM_NOISE_STABILITY_TEST_DEBUG
#define RVL_FEATURE_TEST_SCENE_SEQUENCE
// #define RVL_FEATURE_TEST_PRECISION_RECALL_GRAPH
#define RVL_LOAD_SINGLE_MODEL
// #define PSGM_MATCHES_PROBABILITY_COMPARE
#define PSGM_MATCHES_SCORE_COMPARE
// #define PSGM_LOAD_CTI_FROM_FILE
// #define PSGM_RECOGNITION_VISUALIZE_SCENE
// #define RVLRECOGNITION_DEMO_CLASS_ALIGNMENT
#define RVLPSGM_TRANSPARENCY_AND_COLLISION
#define RVLPSGM_RMSE_CALCULATION
#ifndef RVLVERSION_171125
#define RVLRECOGNITION_DEMO_CLASS_ALIGNMENT
#endif
// #define RVLPSGM_DETERMINE_THRESHOLDS
// #define RVLPSGM_CUDA_ICP
// #define RVLPSGM_ICP_IN_SCENE_CS

#define RVLRECOGNITION_DEMO_FLAG_SAVE_PLY 0x00000001
#define RVLRECOGNITION_DEMO_FLAG_3D_VISUALIZATION 0x00000002
#define RVLRECOGNITION_DEMO_FLAG_VISUALIZE_VN_MODEL 0x00000004
#define RVLRECOGNITION_DEMO_FLAG_VISUALIZE_VN_INSTANCE 0x00000008
#define RVLRECOGNITION_DEMO_FLAG_STEREO_CALIBRATE 0x00000010
#define RVLCAMERA_NONE 0
#define RVLCAMERA_ASTRA 1
#define RVLCAMERA_KINECT 2

// #define  RVLRECOGNITION_DEMO_EVALUATION

// END VIDOVIC

using namespace RVL;

#ifdef RVLPSGM_CUDA_ICP
ICPcudaV1 *pCUDAICPObjv1;
// ICPcudaV2 *pCUDAICPObjv2;
#endif

#ifdef RVLPCL
void GenerateSegmentNeighbourhood(PSGM *psgm, double radius);
void CreateSceneKdTree(PSGM *psgm);
#endif

void CreateParamList(
	CRVLParameterList *pParamList,
	CRVLMem *pMem,
	char **pMeshFileName,
	char **pRGBImageFolder,
	char **pDepthImageFolder,
	char **pPLYFolder,
	char **pTransformationsFolder,
	char **pSceneSequenceFileName, // VIDOVIC
	char **pSceneSequenceSequenceFileName,
	char **pModelSequenceFileName, // VIDOVIC
	char **pCameraParamsFileName,
	char **pStereoCalibrationPath,
	char **pAOFileName,
	char **pModelsInDB,		   // VIDOVIC
	char **pGTFolder,		   // VIDOVIC
	char **pSegmentGTFileName, // Vidovic
	char **pResultsFolder,
	char **pInstanceFileName,
	char **pPrimitiveFileName,
	char **pRotInvarianceInfoFileName,
	DWORD &method,
	DWORD &flags,
	int &iClass,
	int &iMetamodel,
	float &maxMeshTriangleEdgeLen,
	float &SDFSurfaceValue,
	bool &bCreateVisibleSurfaceMesh,
	bool &bSceneBrowser,
	DWORD &cameraType,
	bool &bGT)
{
	pParamList->m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	pParamList->Init();

	pParamData = pParamList->AddParam("SceneFileName", RVLPARAM_TYPE_STRING, pMeshFileName);
	pParamData = pParamList->AddParam("RGBImageFolder", RVLPARAM_TYPE_STRING, pRGBImageFolder);
	pParamData = pParamList->AddParam("DepthImageFolder", RVLPARAM_TYPE_STRING, pDepthImageFolder);
	pParamData = pParamList->AddParam("PLYFolder", RVLPARAM_TYPE_STRING, pPLYFolder);
	pParamData = pParamList->AddParam("TransformationsFolder", RVLPARAM_TYPE_STRING, pTransformationsFolder);
	pParamData = pParamList->AddParam("SceneSequenceFileName", RVLPARAM_TYPE_STRING, pSceneSequenceFileName); // VIDOVIC
	pParamData = pParamList->AddParam("SceneSequenceSequenceFileName", RVLPARAM_TYPE_STRING, pSceneSequenceSequenceFileName);
	pParamData = pParamList->AddParam("ModelSequenceFileName", RVLPARAM_TYPE_STRING, pModelSequenceFileName); // VIDOVIC
	pParamData = pParamList->AddParam("CameraParamsFileName", RVLPARAM_TYPE_STRING, pCameraParamsFileName);
	pParamData = pParamList->AddParam("AOFileName", RVLPARAM_TYPE_STRING, pAOFileName);
	pParamData = pParamList->AddParam("StereoCalibrationPath", RVLPARAM_TYPE_STRING, pStereoCalibrationPath);
	pParamData = pParamList->AddParam("StereoCalibrate", RVLPARAM_TYPE_FLAG, &flags);
	pParamList->AddID(pParamData, "yes", RVLRECOGNITION_DEMO_FLAG_STEREO_CALIBRATE);
	pParamData = pParamList->AddParam("ModelsInDataBase", RVLPARAM_TYPE_STRING, pModelsInDB); // VIDOVIC
	pParamData = pParamList->AddParam("GTFolder", RVLPARAM_TYPE_STRING, pGTFolder);			  // VIDOVIC
	pParamData = pParamList->AddParam("ResultsFolder", RVLPARAM_TYPE_STRING, pResultsFolder);
	pParamData = pParamList->AddParam("SegmentGTFileName", RVLPARAM_TYPE_STRING, pSegmentGTFileName); // Vidovic
	pParamData = pParamList->AddParam("VN.instanceFileName", RVLPARAM_TYPE_STRING, pInstanceFileName);
	pParamData = pParamList->AddParam("PrimitiveFileName", RVLPARAM_TYPE_STRING, pPrimitiveFileName);
	pParamData = pParamList->AddParam("RotInvarianceInfoFileName", RVLPARAM_TYPE_STRING, pRotInvarianceInfoFileName);
	pParamData = pParamList->AddParam("Recognition.method", RVLPARAM_TYPE_ID, &method);
	pParamList->AddID(pParamData, "PSGM", RVLRECOGNITION_METHOD_PSGM);
	pParamList->AddID(pParamData, "RF", RVLRECOGNITION_METHOD_RF); // VIDOVIC
	pParamList->AddID(pParamData, "VN", RVLRECOGNITION_METHOD_VN);
	pParamList->AddID(pParamData, "TG", RVLRECOGNITION_METHOD_TG);
	pParamList->AddID(pParamData, "DDD", RVLRECOGNITION_METHOD_DDD);
	pParamList->AddID(pParamData, "BM", RVLRECOGNITION_METHOD_BM);
	pParamData = pParamList->AddParam("Save PLY", RVLPARAM_TYPE_FLAG, &flags); // VIDOVIC
	pParamList->AddID(pParamData, "yes", RVLRECOGNITION_DEMO_FLAG_SAVE_PLY);   // VIDOVIC
	pParamData = pParamList->AddParam("3D Visualization", RVLPARAM_TYPE_FLAG, &flags);
	pParamList->AddID(pParamData, "yes", RVLRECOGNITION_DEMO_FLAG_3D_VISUALIZATION);
	pParamData = pParamList->AddParam("VN.visualizeModel", RVLPARAM_TYPE_FLAG, &flags);
	pParamList->AddID(pParamData, "yes", RVLRECOGNITION_DEMO_FLAG_VISUALIZE_VN_MODEL);
	pParamData = pParamList->AddParam("VN.visualizeInstance", RVLPARAM_TYPE_FLAG, &flags);
	pParamList->AddID(pParamData, "yes", RVLRECOGNITION_DEMO_FLAG_VISUALIZE_VN_INSTANCE);
	pParamData = pParamList->AddParam("VN.class", RVLPARAM_TYPE_INT, &iClass);
	pParamData = pParamList->AddParam("VN.metamodel", RVLPARAM_TYPE_INT, &iMetamodel);
	pParamData = pParamList->AddParam("MeshBuilder.organizedFastMesh.maxEdgeLen", RVLPARAM_TYPE_FLOAT, &maxMeshTriangleEdgeLen);
	pParamData = pParamList->AddParam("VN.visualization.SDFSurfaceValue", RVLPARAM_TYPE_FLOAT, &SDFSurfaceValue);
	pParamData = pParamList->AddParam("Create visible surface mesh", RVLPARAM_TYPE_BOOL, &bCreateVisibleSurfaceMesh);
	pParamData = pParamList->AddParam("Scene Browser", RVLPARAM_TYPE_BOOL, &bSceneBrowser);
	pParamData = pParamList->AddParam("Camera", RVLPARAM_TYPE_ID, &cameraType);
	pParamList->AddID(pParamData, "NONE", RVLCAMERA_NONE);
	pParamList->AddID(pParamData, "ASTRA", RVLCAMERA_ASTRA); // VIDOVIC
	pParamList->AddID(pParamData, "KINECT", RVLCAMERA_KINECT);
	pParamData = pParamList->AddParam("GT", RVLPARAM_TYPE_BOOL, &bGT);
}

void IOFileNames(
	std::string sceneFolder,
	char *fileName,
	char *RGBImageFolder,
	char *depthImageFolder,
	char *PLYFolder,
	char *transformationsFolder,
	std::string &RGBFileName,
	std::string &depthFileName,
	std::string &PLYFileName,
	std::string &transformationsFileName)
{
	if (RGBImageFolder)
		RGBFileName = sceneFolder + std::string(RGBImageFolder) + RVLFILEPATH_SEPARATOR_ + std::string(fileName);
	else
		RGBFileName = sceneFolder + std::string(fileName);

	if (depthImageFolder)
		depthFileName = sceneFolder + std::string(depthImageFolder) + RVLFILEPATH_SEPARATOR_ + std::string(fileName);
	else
		depthFileName = sceneFolder + std::string(fileName);

	PLYFileName = std::string(fileName, std::string(fileName).rfind(".") + 1) + "ply";

	if (PLYFolder)
		PLYFileName = sceneFolder + std::string(PLYFolder) + RVLFILEPATH_SEPARATOR_ + PLYFileName;
	else
		PLYFileName = sceneFolder + PLYFileName;

	transformationsFileName = std::string(fileName, std::string(fileName).rfind(".") + 1) + "yaml";

	if (transformationsFolder)
		transformationsFileName = sceneFolder + std::string(transformationsFolder) + RVLFILEPATH_SEPARATOR_ + transformationsFileName;
	else
		transformationsFileName = sceneFolder + transformationsFileName;
}

unsigned char *img;
int16_t *imgD;

int main(int argc, char **argv)
{
	// Create memory storage.

	CRVLMem mem0; // permanent memory

	mem0.Create(1000000000);
	// mem0.Create(100000000000); //VIDOVIC

	CRVLMem mem; // cycle memory

	mem.Create(1000000000);
	// mem.Create(100000000000); //VIDOVIC

	// Read parameters from a configuration file.

	char cfgSelectionFileName[] = "RVLRecognitionDemo.cfg";

	char *cfgFileName = ReadConfigurationFile(cfgSelectionFileName);

	if (cfgFileName == NULL)
		return 1;

	printf("Configuration file: %s\n", cfgFileName);

	char *sceneMeshFileName = NULL;
	char *RGBImageFolder = NULL;
	char *depthImageFolder = NULL;
	char *PLYFolder = NULL;
	char *transformationsFolder = NULL;
	char *sceneSequenceFileName = NULL; // VIDOVIC
	char *sceneSequenceSequenceFileName = NULL;
	char *modelSequenceFileName = NULL; // VIDOVIC
	char *cameraParamsFileName = NULL;
	char *AOFileName = NULL;
	char *stereoCalibrationPath = NULL;
	char *modelsInDB = NULL; // VIDOVIC
	char *GTFolder = NULL;	 // VIDOVIC
	char *ResultsFolder = NULL;
	char *segmentGTFileName = NULL; // Vidovic
	char *instanceFileName = NULL;
	char *primitiveFileName = NULL;
	char *rotInvarianceInfoFileName = NULL;
	DWORD method = RVLRECOGNITION_METHOD_PSGM;
	// DWORD method = RVLRECOGNITION_METHOD_RF; //VIDOVIC
	int iClass;
	int iMetamodel = 0;
	float maxMeshTriangleEdgeLen = -1.0f;
	float SDFSurfaceValue = 0.0f;
	bool bCreateVisibleSurfaceMesh = false;
	bool bSceneBrowser = false;
	DWORD cameraType = RVLCAMERA_NONE;
	bool bGT = true;
	bool bMeshBuilder = false;

	DWORD flags = 0x00000000; // VIDOVIC

	CRVLParameterList ParamList;

	CreateParamList(&ParamList,
					&mem0,
					&sceneMeshFileName,
					&RGBImageFolder,
					&depthImageFolder,
					&PLYFolder,
					&transformationsFolder,
					&sceneSequenceFileName,
					&sceneSequenceSequenceFileName,
					&modelSequenceFileName,
					&cameraParamsFileName,
					&AOFileName,
					&stereoCalibrationPath,
					&modelsInDB,
					&GTFolder,
					&segmentGTFileName,
					&ResultsFolder,
					&instanceFileName,
					&primitiveFileName,
					&rotInvarianceInfoFileName,
					method,
					flags,
					iClass,
					iMetamodel,
					maxMeshTriangleEdgeLen,
					SDFSurfaceValue,
					bCreateVisibleSurfaceMesh,
					bSceneBrowser,
					cameraType,
					bGT); // VIDOVIC

	ParamList.LoadParams(cfgFileName);

	// Create mesh builder.

#ifdef RVLPCL
	PCLMeshBuilder meshBuilder;

	meshBuilder.Create(cfgFileName, &mem0);

	int w = meshBuilder.width;
	int h = meshBuilder.height;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC(new pcl::PointCloud<pcl::PointXYZRGBA>(w, h));

	meshBuilder.PC = PC;

	if (bCreateVisibleSurfaceMesh)
		meshBuilder.flags |= RVLPCLMESHBUILDER_FLAG_VISIBLE_SURFACE;

	maxMeshTriangleEdgeLen = meshBuilder.organizedFastMeshMaxEdgeLen;
#else
	if (bMeshBuilder)
	{
		printf("ERROR: Mesh builder is not available because PCL is not linked!\n");
		return 1;
	}
#endif

	// Load camera parameters.

	Camera camera;
	LoadCameraParametersFromFile(cfgFileName, camera, &mem0);

	// Input and output directories.

	std::string sceneFolder; // = std::string(sceneSequenceFileName, std::string(sceneSequenceFileName).rfind(pathSeparator) + 1);
	std::string RGBFileName, depthFileName, PLYFileName, transformationsFileName;

	// Generate meshes in PLY format from images.

	if (flags & RVLRECOGNITION_DEMO_FLAG_SAVE_PLY)
	{
		Mesh mesh;

		FileSequenceLoader sceneSequence;
		char fileName[200];

		sceneSequence.Init(sceneSequenceFileName);
		sceneFolder = std::string(sceneSequenceFileName, std::string(sceneSequenceFileName).rfind(RVLFILEPATH_SEPARATOR_) + 1);
		std::string RGBFileName, depthFileName, PLYFileName;

		while (sceneSequence.GetNextName(fileName))
		{
			IOFileNames(sceneFolder, fileName, RGBImageFolder, depthImageFolder, PLYFolder, transformationsFolder, RGBFileName, depthFileName, PLYFileName, transformationsFileName);
#ifdef RVLPCL
			if (bMeshBuilder)
				LoadMesh(&meshBuilder, (char *)RGBFileName.data(), &mesh, true, (char *)PLYFileName.data(), (char *)depthFileName.data());
			else
#endif
			{
				printf("Creating PLY from RGBD image %s.\n", fileName);
				MESH::RGBD2PLY(RGBFileName, depthFileName, camera, maxMeshTriangleEdgeLen, PLYFileName);
			}
		}

		RVL_DELETE_ARRAY(sceneMeshFileName);
		RVL_DELETE_ARRAY(sceneSequenceFileName);
		RVL_DELETE_ARRAY(modelSequenceFileName);
		RVL_DELETE_ARRAY(cameraParamsFileName);
		RVL_DELETE_ARRAY(stereoCalibrationPath)

		return 0;
	}

#ifdef RVLOPENNI
	// Create object for camera:
	CRVLKinect kinect;
	Astra astra;

	if (cameraType == RVLCAMERA_ASTRA)
	{
		astra.InitAstra();

		// Orbec:
#ifdef RVLPCL
		meshBuilder.camera.depthFu0 = 519.14016473372533;
		meshBuilder.camera.depthUc0 = 316.36700343463434;
		meshBuilder.camera.depthFv0 = 518.95999218830843;
		meshBuilder.camera.depthVc0 = 250.72443412270945;
#endif
	}

	if (cameraType == RVLCAMERA_KINECT)
	{
		kinect.Init();

		kinect.m_scale = 1;

		kinect.RegisterDepthToColor(true);

		// Kinect:
#ifdef RVLPCL
		meshBuilder.camera.depthFu0 = 584.70194597700402;
		meshBuilder.camera.depthUc0 = 318.55964537649561;
		meshBuilder.camera.depthFv0 = 585.70332900816618;
		meshBuilder.camera.depthVc0 = 256.14501544470505;
#endif
	}
#endif

#ifdef RVLPCL
	meshBuilder.camera.depthFu = meshBuilder.camera.depthFu0;
	meshBuilder.camera.depthUc = meshBuilder.camera.depthUc0;
	meshBuilder.camera.depthFv = meshBuilder.camera.depthFv0;
	meshBuilder.camera.depthVc = meshBuilder.camera.depthVc0;
#endif

	// Create segment GT file name.

	if (segmentGTFileName == NULL)
	{
		segmentGTFileName = new char[200];
		segmentGTFileName = "C:\\RVL\\segmentGT.txt";
	}

	// Initialize surfel detection

	SurfelGraph surfels;

	surfels.pMem = &mem;

	surfels.CreateParamList(&mem0);

	surfels.ParamList.LoadParams(cfgFileName);

	PlanarSurfelDetector surfelDetector;

	surfelDetector.CreateParamList(&mem0);

	surfelDetector.ParamList.LoadParams(cfgFileName);

	// VIDOVIC
	// initialize mesh noiser
	MeshNoiser noiser;

	noiser.SetParam(1, 0.05);
	// END VIDOVIC

	// Initialize visualization

	unsigned char SelectionColor[3];

	SelectionColor[0] = 0;
	SelectionColor[1] = 255;
	SelectionColor[2] = 0;

	Visualizer visualizer;

	visualizer.Create();

	// Main part - recognition.

	if (method == RVLRECOGNITION_METHOD_RF)
	{
		// Initialize recognition.

		RFRecognition recognition;

		recognition.CreateParamList(&mem0);

		recognition.ParamList.LoadParams(cfgFileName);

		recognition.pMem0 = &mem0;
		recognition.pMem = &mem;

		recognition.pSurfels = &surfels;

		recognition.pSurfelDetector = &surfelDetector;

		// Training or recognition (depending on mode).

		if (recognition.mode == RVLRECOGNITION_MODE_TRAINING)
			recognition.CreateModelDatabase();
		else if (recognition.mode == RVLRECOGNITION_MODE_RECOGNITION)
		{
			if (!recognition.LoadModelDatabase())
				return 1;

			Mesh mesh;

			// VIDOVIC
#ifdef RVL_COORDINATE_SYSTEM_NOISE_STABILITY_TEST

			recognition.CoordinateSystemNoiseStabilityTest(sceneMeshFileName, noiser, 0);

#endif // RVL_COORDINATE_SYSTEM_NOISE_STABILITY_TEST

#ifdef RVL_FEATURE_TEST_SCENE_SEQUENCE

			recognition.FeatureTestSceneSequence(sceneSequenceFileName, noiser);

#endif // RVL_FEATURE_TEST_SCENE_SEQUENCE

#ifdef RVL_FEATURE_TEST_PRECISION_RECALL_GRAPH

			recognition.FeatureTestPrecisionRecallGraph(sceneMeshFileName, sceneSequenceFileName, noiser);

#endif // RVL_FEATURE_TEST_PRECISION_RECALL_GRAPH

#ifdef RVL_LOAD_SINGLE_MODEL

			// VIDOVIC
			RECOG::Hypothesis *pBestHypothesis = NULL;
			float V[3], theta;
			float distance;
			// END VIDOVIC

			mesh.LoadPolyDataFromPLY(sceneMeshFileName);

			recognition.FindObjects(&mesh);

			// VIDOVIC
			recognition.FindBestHypothesis(&pBestHypothesis);

			GetAngleAxis(pBestHypothesis->R, V, theta);
			GetDistance(pBestHypothesis->t, distance);

			FILE *fpHypothesisErrorDebug = NULL;

			fpHypothesisErrorDebug = fopen("C:\\RVL\\Debug\\hypothesisErrorDebug.txt", "w");
			// END VIDOVIC

			FILE *fpInterpretation = fopen("C:\\RVL\\Debug\\interpretation.txt", "w");

			RECOG::Hypothesis *pHypothesis = recognition.sceneInterpretation.pFirst;

			while (pHypothesis)
			{
				RECOG::WriteHypothesis(fpInterpretation, pHypothesis);

				// VIDOVIC
				GetAngleAxis(pHypothesis->R, V, theta);
				GetDistance(pHypothesis->t, distance);

				RECOG::WriteHypothesisError(fpHypothesisErrorDebug, pHypothesis, distance, theta * 180 / PI);
				// END VIDOVIC

				pHypothesis = pHypothesis->pNext;
			}

			fclose(fpInterpretation);
			fclose(fpHypothesisErrorDebug); // VIDOVIC

#endif // RVL_LOAD_SINGLE_MODEL
	   // END VIDOVIC

			// Visualization

			// surfels.NodeColors(SelectionColor); //VIDOVIC
			// recognition.InitDisplay(&visualizer, &mesh);
			// recognition.Display();
			// visualizer.Run();
		}
	} // if (method == RVLRECOGNITION_METHOD_RF)
	else if (method == RVLRECOGNITION_METHOD_PSGM)
	{
#ifdef RVLPCL
		// Initialize recognition.

		PSGM recognition;

		recognition.CreateParamList(&mem0);

		recognition.ParamList.LoadParams(cfgFileName);

		// recognition.Create();

		recognition.pMem = &mem;
		recognition.pMem0 = &mem0;

#ifdef RVLPCL
		recognition.vpMeshBuilder = &meshBuilder;
		recognition.LoadMesh = LoadMesh;
#endif

		recognition.pSurfels = &surfels;

		recognition.pSurfelDetector = &surfelDetector;

		recognition.MTGSet.pMem = recognition.pMem0;

		recognition.Init(cfgFileName);

		recognition.resultsFolder = ResultsFolder;

		if (recognition.mode == RVLRECOGNITION_MODE_TRAINING)
		{
			recognition.bDetectGroundPlane = false;

			recognition.Learn(modelSequenceFileName, &visualizer); // Vidovic
		}
		else if (recognition.mode == RVLRECOGNITION_MODE_RECOGNITION)
		{
			// Eigen::MatrixXf nI = recognition.ConvexTemplatenT();
			// float dI[66];
			// for (int i = 0; i < 66; i++) dI[i] = 1;
			// recognition.RVLPSGInstanceMesh(nI, dI);

#ifdef RVLPCL
			recognition.camera.fu = meshBuilder.camera.depthFu;
			recognition.camera.fv = meshBuilder.camera.depthFv;
			recognition.camera.uc = meshBuilder.camera.depthUc;
			recognition.camera.vc = meshBuilder.camera.depthVc;
			recognition.camera.w = meshBuilder.camera.w;
			recognition.camera.h = meshBuilder.camera.h;
#endif

			if (!recognition.bSegmentationOnly)
			{
				recognition.LoadModelDataBase(); // Vidovic
				// recognition.LoadModelMeshDB(modelSequenceFileName, &recognition.vtkModelDB, false, 0.4);

				if (recognition.problem == RVLRECOGNITION_PROBLEM_CLASSIFICATION)
				{
					recognition.LoadTransformationMatrices();
					recognition.LoadReferentModels();
				}
			}

#ifdef RVLPSGM_ICP
			if (recognition.problem == RVLRECOGNITION_PROBLEM_SHAPE_INSTANCE_DETECTION)
				recognition.LoadModelMeshDB(modelSequenceFileName, &recognition.vtkModelDB, false, 0.4);
#endif

			Mesh mesh;

			// Vidovic
			char filePath[200];

			char *CTIFileName = NULL;

			recognition.pTimer = new CRVLTimer;

			FileSequenceLoader sceneSequence;

			sceneSequence.Init(sceneSequenceFileName);

			// recognition.pSurfels->bContactEdgeVertices = true;

			recognition.pECCVGT->Init(sceneSequence, GTFolder, modelsInDB);

			// recognition.pECCVGT->SaveGTFile("D:\\ARP3D\\TUW_GT.txt");

			// FILE *fpHypothesisEvaluation = fopen("D:\\ARP3D\\compare_TNM_Valid_TMP.txt", "w");

			// FILE *fpLog = fopen("D:\\ARP3D\\evaluationLog.txt", "w");

			std::string resultsFolderName = std::string(ResultsFolder);

			FILE *fpPoseError = fopen((resultsFolderName + "\\poseError.txt").data(), "w");

			FILE *fpnotFirstInfo = fopen((resultsFolderName + "\\notFirstInfo.txt").data(), "w");

			FILE *fpnotFirstPoseErr = fopen((resultsFolderName + "\\notFirstInfo.txt").data(), "w");

			// recognition.pECCVGT->SaveGTFile("C:\\RVL\\ExpRez\\TUW_GT.txt");

			FILE *fpHypothesisEvaluation = fopen((resultsFolderName + "\\compare_TNM_Valid_TMP.txt").data(), "w");

			FILE *fpLog = fopen((resultsFolderName + "\\evaluationLog.txt").data(), "w");

			FILE *fpRMSE = fopen((resultsFolderName + "\\RMSE.txt").data(), "w");

			FILE *fpTime = fopen((resultsFolderName + "\\time.txt").data(), "w");

			FILE *fpForegroundObject = fopen((std::string(ResultsFolder) + "\\detected_object.m").data(), "w");

			fclose(fpForegroundObject);

#ifdef RVLPSGM_CUDA_ICP
			// create object for CUDA ICP
			pCUDAICPObjv1 = new ICPcudaV1();
			pCUDAICPObjv1->fx = 525 / 2.0;
			pCUDAICPObjv1->fy = 525 / 2.0;
			pCUDAICPObjv1->cx = 320 / 2.0;
			pCUDAICPObjv1->cy = 240 / 2.0;
			pCUDAICPObjv1->width = 320;
			pCUDAICPObjv1->height = 240;
			pCUDAICPObjv1->noCudaThreads = 128;
			pCUDAICPObjv1->noCudaBlocks = 32;
			pCUDAICPObjv1->searchDistanceThr = 0.05;
			pCUDAICPObjv1->searchAngleThreshold = 0.64278760968f;
			pCUDAICPObjv1->InitICP();
			pCUDAICPObjv1->SetICPIterations(15, 10, 10);
#endif

#ifdef RVLPSGM_RMSE_CALCULATION
			// Load models without decimation (used for calculating RMSE)
			// recognition.LoadModelMeshDB(modelSequenceFileName, &recognition.vtkRMSEModelDB, false);
			recognition.vtkRMSEModelDB = recognition.vtkModelDB;
#endif

			// allow user to set real-time priority in task manager - time measurement TPAMI18
			// printf("Press any key to start experiment!\n");
			// system("pause");

			recognition.LoadCompleteSegmentGT(sceneSequence);

			recognition.falseHypothesesFileName = RVLCreateString((char *)((resultsFolderName + "\\falseHypotheses.txt").c_str()));

			FILE *fpFalseHypotheses = fopen(recognition.falseHypothesesFileName, "w");

			fclose(fpFalseHypotheses);

			recognition.TPHypothesesCTIRankFileName = RVLCreateString((char *)((resultsFolderName + "\\TPHypothesesCTIRank.txt").c_str()));

			FILE *fpTPHypothesesCTIRank = fopen(recognition.TPHypothesesCTIRankFileName, "w");

			fclose(fpTPHypothesesCTIRank);

			int command = 1;

			// LARGE_INTEGER ctr1, ctr2, freq;
			// LARGE_INTEGER ctr1_, ctr2_, freq_;

#ifdef RVLVN_TIME_MESUREMENT
			LARGE_INTEGER CNTREvaluation3START, CNTREvaluation3END;
			LARGE_INTEGER CNTRHypothesisPruningSTART, CNTRHypothesisPruningEND;

			LARGE_INTEGER frequency;
#endif

			// CUDA MATRICES PREPARATION - TEST
			float *W = NULL, *VA = NULL, *Ds = NULL, *Rs = NULL, *ts = NULL;
			int *V = NULL;
			float *Dm = NULL, *Rm_inv = NULL, *tm_inv = NULL;

			int sceneNo = 0;
			while (true)
			{
				if (cameraType == RVLCAMERA_ASTRA || cameraType == RVLCAMERA_KINECT)
				{
#ifdef RVLOPENNI
					RVL::Array2D<short int> depthImage;
					IplImage *GSImage, *RGBImage, *depthDisplayImage;
					depthImage.Element = NULL;
					RGBImage = NULL;
					GSImage = NULL;
					depthDisplayImage = NULL;
					depthImage.w = 640;
					depthImage.h = 480;
					if (cameraType == RVLCAMERA_ASTRA)
					{
						int w = 640;
						int h = 480;
						astra.GetImageFromAstra();
						depthImage.Element = (short *)(astra.depthF.data);
						RGBImage = new IplImage(astra.ImageF);
					}
					if (cameraType == RVLCAMERA_KINECT)
					{

						openni::VideoFrameRef depthFrame;
						openni::VideoFrameRef RGBFrame;

						int nPix = depthImage.w * depthImage.h;

						RVL_DELETE_ARRAY(depthImage.Element);
						depthImage.Element = new short int[nPix];

						int wRGB = depthImage.w / 2;
						int hRGB = depthImage.h / 2;

						cvReleaseImage(&RGBImage);
						RGBImage = cvCreateImage(cvSize(wRGB, hRGB), IPL_DEPTH_8U, 3);

						cvReleaseImage(&GSImage);
						GSImage = cvCreateImage(cvSize(wRGB, hRGB), IPL_DEPTH_8U, 1);

						cvReleaseImage(&depthDisplayImage);
						depthDisplayImage = cvCreateImage(cvSize(depthImage.w, depthImage.h), IPL_DEPTH_8U, 3);

						kinect.GetImages(depthImage.Element, RGBImage, NULL, GSImage, RVLKINECT_DEPTH_IMAGE_FORMAT_1MM, 0, &depthFrame, &RGBFrame);

						// DisplayDisparityMap(depthImage, (unsigned char *)(depthDisplayImage->imageData), false, RVLRGB_DEPTH_FORMAT_1MM);
						// cvShowImage("depth image", depthDisplayImage);
						// cv::waitKey();
						RVL_DELETE_ARRAY(depthImage.Element);
						cvReleaseImage(&RGBImage);
						cvReleaseImage(&GSImage);
						cvReleaseImage(&depthDisplayImage);
					}

					printf("Creating mesh from depth image.\n");
					Mesh mesh;
					std::string path = std::string("D:\\ARP3D\\3DNet_dataset\\Cat10_TestDatabase\\PLY\\scene") + std::to_string(sceneNo) + ".ply";
					char *str = &path[0u];

#ifdef RVLPCL
					CreateMesh(&meshBuilder, &depthImage, RGBImage, &mesh, true, str);
#endif
					// CreateMesh(&meshBuilder, &depthImage, RGBImage, &mesh, true, "D:\\ARP3D\\3DNet_dataset\\Cat10_TestDatabase\\PLY\\scene.ply");
					printf("Mesh created.\n");
					sceneNo++;

					std::strcpy(filePath, str);
#ifdef RVLPCL
					LoadMesh(&meshBuilder, filePath, &mesh, false);
#endif
#else
					printf("ERROR: OpenNI requred!\n");
#endif
				}
				else
				{
					if (bSceneBrowser)
					{
						switch (command)
						{
						case 0:
							recognition.SceneBackward();

							break;
						case 1:
							if (!sceneSequence.GetNextPath(filePath))
								break;

							break;
						case 2:
							printf("Select scene:\n");

							int iScene;

							scanf("%d", &iScene);

							recognition.SetScene(iScene);

							sceneSequence.GetFilePath(iScene, filePath);
						}
					}
					else if (!sceneSequence.GetNextPath(filePath))
						break;

						// mesh.LoadPolyDataFromPLY(filePath);
#ifdef RVLPCL
					LoadMesh(&meshBuilder, filePath, &mesh, false);
#endif
				}

				recognition.ParamList.LoadParams(cfgFileName);

				// QueryPerformanceCounter((LARGE_INTEGER *)&ctr1);

				// Visualize model CTIs - images creation for TPAMI18
				// recognition.VisualizeCTI(&visualizer, 0, true);

				// recognition.InitDisplay(&visualizer, &mesh, SelectionColor);
				// for (int i = 0; i < 35; i++)
				//{
				//	visualizer.renderer->RemoveAllViewProps();
				//	recognition.VisualizeWholeModelCTI(i);
				// }
				// recognition.VisualizeDBModels(modelSequenceFileName, &visualizer);

				// recognition.VisualizeModel("F:\\Projekti\\ARP3D\\Auxiliary\\ECCV_dataset\\pcd_files\\frame_20111220T115946.415206.ply", &visualizer);
				// recognition.VisualizeModel("F:\\Projekti\\ARP3D\\Auxiliary\\ECCV_dataset\\pcd_files\\frame_20111220T120147.910222.ply", &visualizer);

				// recognition.VisualizeCTI(73, true, &visualizer);

				// recognition.VisualizeCTI(622, true, &visualizer);
				// recognition.VisualizeCTI(697, true, &visualizer);
				// recognition.VisualizeCTI(718, true, &visualizer);

				// recognition.VisualizeCTI(1189, true, &visualizer);

				// recognition.VisualizeCTI(1777, true, &visualizer);
				// END Visualize model CTIs

				printf("Scene %s...\n", filePath);

				recognition.SetSceneFileName(filePath);
				// recognition.InterpretCTIS(&mesh);

#ifdef PSGM_LOAD_CTI_FROM_FILE
				RVLCopyString(filePath, &CTIFileName);

				sprintf(RVLGETFILEEXTENSION(CTIFileName), "cti");

				recognition.LoadCTI(CTIFileName);

				recognition.Match();

				surfels.NodeArray.n = 0;

				recognition.clusters.n = 0;
#else

#ifdef NEVER
				// Generate scene depth
				//************************************************************************************************************
				// Vidovic commented on 21.07.2017.
				// because function PSGM::CreateDilatedDepthImage(); is called inside PSGM::FilterHypothesesUsingTransparency()
				//************************************************************************************************************
				// double point[3];
				// int u, v;
				// cv::Mat depth(480, 640, CV_16UC1, cv::Scalar::all(0));
				// for (int i = 0; i < mesh.pPolygonData->GetNumberOfPoints(); i++)
				//{
				//	mesh.pPolygonData->GetPoint(i, point);
				//	if ((point[0] == 0) && (point[1] == 0) && (point[2] == 0))
				//		continue;
				//	v = floor(float(i) / 640);
				//	u = i - v * 640;
				//	depth.at<uint16_t>(v, u) = (uint16_t)(point[2] * 1000); //in milimeters
				// }
				////Postprocessing
				// for (int y = 0; y < depth.rows; y++)
				//{
				//	for (int x = 0; x < depth.cols; x++)
				//	{
				//		if (depth.at<uint16_t>(y, x) == 0)
				//			depth.at<uint16_t>(y, x) = 10000; //in milimeters
				//	}
				// }
				// cv::Mat elementE = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(17, 17));
				// cv::erode(depth, depth, elementE);
				////Set PSGM depth
				// recognition.depthImg = (unsigned short*)depth.data;
				//************************************************************************************************************
				// END Vidovic commented
				//************************************************************************************************************

				/*cv::Mat depthShow(480, 640, CV_8UC1);
				double minVal, maxVal;
				cv::minMaxLoc(depth, &minVal, &maxVal);
				depth.convertTo(depthShow, CV_8U, -255.0f / maxVal, 255.0f);
				cv::imshow("depth image", depthShow);
				cv::waitKey();*/

				///////////TEST/////////
				///*std::fstream fileS("eccv_frame_20111221T142636.413299_depth.txt", std::fstream::out);
				// double point[3];
				// int u, v;
				// for (int i = 0; i < mesh.pPolygonData->GetNumberOfPoints(); i++)
				//{
				//	mesh.pPolygonData->GetPoint(i, point);
				//	if ((point[0] == 0) && (point[1] == 0) && (point[2] == 0))
				//		continue;
				//	v = floor(float(i) / 640);
				//	u = i - v * 640;
				//	fileS << u << " " << v << " " << point[0] << " " << point[1] << " " << point[2] << std::endl;
				// }
				// fileS.close();*/
				////Generate scene depth
				// double point[3];
				// int u, v;
				// cv::Mat origDepth(480, 640, CV_16UC1, cv::Scalar::all(0));
				// for (int i = 0; i < mesh.pPolygonData->GetNumberOfPoints(); i++)
				//{
				//	mesh.pPolygonData->GetPoint(i, point);
				//	if ((point[0] == 0) && (point[1] == 0) && (point[2] == 0))
				//		continue;
				//	v = floor(float(i) / 640);
				//	u = i - v * 640;
				//	origDepth.at<uint16_t>(v, u) = (uint16_t)(point[2] * 1000); //in milimeters
				// }
				//// Initialize VTK.
				// vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
				// vtkSmartPointer<vtkRenderWindow> renWin = vtkSmartPointer<vtkRenderWindow>::New();
				///*vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
				// interactor->SetRenderWindow(renWin);
				// vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
				// interactor->SetInteractorStyle(style);*/
				// renWin->OffScreenRenderingOn(); //OFF-SCREEN RENDERING
				// renWin->AddRenderer(renderer);
				// renWin->SetSize(640, 480); //HARDCODED 640X480 IMAGE

				////adding polydata actor
				// vtkSmartPointer<vtkPolyDataMapper>	map = vtkSmartPointer<vtkPolyDataMapper>::New();
				// map->SetInputData(mesh.pPolygonData);
				// vtkSmartPointer<vtkActor> act = vtkSmartPointer<vtkActor>::New();
				// act->SetMapper(map);
				// renderer->AddActor(act);
				////renWin->Render();
				////interactor->Start();

				////find zbounds
				// double *bounds;
				// mesh.pPolygonData->GetPoints()->ComputeBounds(); //just in case
				// bounds = mesh.pPolygonData->GetPoints()->GetBounds(); // (Xmin, Xmax) = (bounds[0], bounds[1]), (Ymin, Ymax) = (bounds[2], bounds[3]), (Zmin, Zmax) = (bounds[4], bounds[5])
				// if (bounds[4] == 0.0)
				//	bounds[4] = 0.4; //0.4m
				// vtkSmartPointer<vtkCamera> camera = CreateVTKCamera_GenericKinect_1(bounds[4], bounds[5]);
				// cv::Mat bufferDepth;
				// for (int i = 0; i < 10; i++)
				//{
				//	LARGE_INTEGER d_ctr1, d_ctr2, d_freq;
				//	QueryPerformanceCounter((LARGE_INTEGER *)&d_ctr1);
				//	bufferDepth = GenerateVTKDepthImage(renWin, camera, 640, 480);// GenerateVTKDepthImage_Kinect(renWin, bounds[4], bounds[5]);
				//	QueryPerformanceCounter((LARGE_INTEGER *)&d_ctr2);
				//	QueryPerformanceFrequency((LARGE_INTEGER *)&d_freq);
				//	float d_timevalue = (d_ctr2.QuadPart - d_ctr1.QuadPart) * 1000.0 / d_freq.QuadPart;
				//	std::cout << "Depth gen vrijeme: " << d_timevalue << std::endl;
				// }
				////show
				// cv::Mat depthShow(480, 640, CV_8UC1);
				// double minVal, maxVal;
				////FilterImage(bufferDepth);
				// cv::minMaxLoc(bufferDepth, &minVal, &maxVal);
				// bufferDepth.convertTo(depthShow, CV_8U, -255.0f / maxVal, 255.0f);
				// cv::imshow("Rendered depth image", depthShow);
				////cv::imwrite("renderedDepthF.png", bufferDepth);
				///*cv::Mat bufferDepthBlur(480, 640, CV_16UC1, cv::Scalar::all(0));
				// cv::Mat depthShowBlur(480, 640, CV_8UC1);
				// cv::GaussianBlur(bufferDepth, bufferDepthBlur, cv::Size(9, 9), 0, 0);
				// cv::minMaxLoc(bufferDepthBlur, &minVal, &maxVal);
				// bufferDepthBlur.convertTo(depthShowBlur, CV_8U, -255.0f / maxVal, 255.0f);
				// cv::imshow("Rendered depth (blur) image", depthShowBlur);*/
				////Original depth
				// cv::minMaxLoc(origDepth, &minVal, &maxVal);
				// cv::Mat depthOrigShow(480, 640, CV_8UC1);
				// origDepth.convertTo(depthOrigShow, CV_8U, -255.0f / maxVal, 255.0f);
				// cv::imshow("Original depth", depthOrigShow);
				////cv::imwrite("origDepth.png", origDepth);
				////Dilate original depth
				// cv::Mat elementD = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(20,20));
				// cv::Mat elementE = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(11, 11));
				// cv::Mat origDepth_D(480, 640, CV_16UC1, cv::Scalar::all(0));
				// LARGE_INTEGER d_ctr1, d_ctr2, d_freq;
				// QueryPerformanceCounter((LARGE_INTEGER *)&d_ctr1);
				// for (int y = 0; y < origDepth.rows; y++)
				//{
				//	for (int x = 0; x < origDepth.cols; x++)
				//	{
				//		if (origDepth.at<uint16_t>(y, x) == 0)
				//			origDepth.at<uint16_t>(y, x) = 10000; //in milimeters
				//	}
				// }
				////cv::dilate(origDepth, origDepth_D, elementD);
				// cv::erode(origDepth, origDepth_D, elementE);
				// QueryPerformanceCounter((LARGE_INTEGER *)&d_ctr2);
				// QueryPerformanceFrequency((LARGE_INTEGER *)&d_freq);
				// float d_timevalue = (d_ctr2.QuadPart - d_ctr1.QuadPart) * 1000.0 / d_freq.QuadPart;
				// std::cout << "Dilate vrijeme: " << d_timevalue << std::endl;

				// cv::Mat depthOrigShow_D(480, 640, CV_8UC1);
				// cv::minMaxLoc(origDepth_D, &minVal, &maxVal);
				// origDepth_D.convertTo(depthOrigShow_D, CV_8U, -255.0f / maxVal, 255.0f);
				// cv::imshow("Original depth (dilated)", depthOrigShow_D);

				//////show the difference between rendered depth and original
				////cv::Mat depthDifference(480, 640, CV_16UC1, cv::Scalar::all(0));
				////cv::absdiff(bufferDepth, origDepth, depthDifference);
				////cv::minMaxLoc(depthDifference, &minVal, &maxVal);
				////cv::Mat depthDifferenceShow(480, 640, CV_8UC1);
				////depthDifference.convertTo(depthDifferenceShow, CV_8U, -255.0f / maxVal, 255.0f);
				////cv::imshow("Difference", depthDifferenceShow);

				// cv::waitKey();
				////interactor->Start();
				////
#endif

				mem.Clear();

				recognition.segmentGTLoaded = false;
				recognition.createSegmentGT = false;
				recognition.visualizeTPHypotheses = false;

				if (flags & RVLRECOGNITION_DEMO_FLAG_3D_VISUALIZATION)
				{
					visualizer.renderer->RemoveAllViewProps();
					recognition.InitDisplay(&visualizer, &mesh, SelectionColor);
				}

				// ONLY FOR CREATING IMAGES FOR THE PAPER
				// recognition.VisualizeDBModels(modelSequenceFileName, &visualizer);
				//

				recognition.Interpret(&mesh);

				//// Only for debugging purpose!!!

				// int iMatch = recognition.bestSceneSegmentMatches2.Element[0].Element[0].idx;

				// recognition.SaveHypothesisProjection(iMatch);

				// recognition.SaveSubsampledScene();

				// recognition.SaveMatches();

#ifdef PSGM_RECOGNITION_VISUALIZE_SCENE
				// Visualize currennt scene (close visualizer window by pressing 'q' key)
				surfels.NodeColors(SelectionColor);
				recognition.InitDisplay(&visualizer, &mesh, SelectionColor);
				recognition.Display();
				visualizer.Run();

				visualizer.renderer->RemoveAllViewProps();
#endif
				if (recognition.problem == RVLRECOGNITION_PROBLEM_CLASSIFICATION)
				{
					// Save the segmentation results to a file.

					char *objectMapFileName = RVLCreateFileName(filePath, ".ply", -1, ".objmap.png");

					cv::Mat objectMask(recognition.pMesh->height, recognition.pMesh->width, CV_8UC1);

					recognition.pObjects->ObjectMapMask(&objectMask);

					// cv::imshow("Object mask", objectMask);

					cv::imwrite(objectMapFileName, objectMask);

					delete[] objectMapFileName;

					cv::waitKey();
				}
#endif // #ifndef PSGM_LOAD_CTI_FROM_FILE
	   // Evaluate CTI match
	   // recognition.EvaluateMatchesByScore(fpHypothesisEvaluation, fpLog, 7);

				printf("Scene %s...finished!\n\n", filePath);

				// mesh.LoadPolyDataFromPLY(filePath);
				// LoadMesh(&meshBuilder, filePath, &mesh, false);

				if (flags & RVLRECOGNITION_DEMO_FLAG_3D_VISUALIZATION)
				{
					surfels.NodeColors(SelectionColor);
					recognition.Display();
				}

				////NEW FILKO - TEST COLLISION CONSENSUS
				// std::vector<int> conHyp = recognition.GetHypothesesCollisionConsensus(20);
				// for (int i = 0; i < conHyp.size(); i++)
				//{
				//	recognition.AddOneModelToVisualizer(&visualizer, conHyp.at(i), 0, false, true);
				// }

				/*QueryPerformanceCounter((LARGE_INTEGER *)&ctr1_);*/

#ifdef RVLPSGM_ICP
				if (recognition.bICP)
				{
					// recognition.CalculateICPCost(PCLICP, PCLICPVariants::Point_to_plane, &kdtree);

					// Timer ICP START
					QueryPerformanceCounter((LARGE_INTEGER *)&CNTREvaluation3START);

#ifdef RVLPCL
					GenerateSegmentNeighbourhood(&recognition, 0.2);
#endif
					// recognition.CalculateNNCost(&visualizer, PCLICP, PCLICPVariants::Point_to_plane);

					// TEST RVLPSGM_MATCHCTI_MATCH_MATRIX
#ifdef RVLVERSION_171125
#ifdef RVLPSGM_CUDA_ICP
					// CUDA ICP
					pCUDAICPObjv1->SetIcpModel(recognition.pSubsampledSceneDepthImage, Eigen::Matrix4f::Identity());

					// cv::Mat depthImage(240, 320, CV_16UC1);
					// memcpy(depthImage.data, recognition.pSubsampledSceneDepthImage, 320 * 240 * sizeof(unsigned short));
					// cv::imwrite("C:\\RVL\\scena.png", depthImage);

					recognition.ICP(RunCUDAICPv1, recognition.bestSceneSegmentMatches2);
#else
					// Create KDTree
					// CreateSceneKdTree(&recognition);

					// QueryPerformanceCounter((LARGE_INTEGER *)&ctr1_);

#ifdef RVLPSGM_ICP_IN_SCENE_CS
					recognition.ICP_refined_cs_scene(PCLICP, PCLICPVariants::Point_to_plane, recognition.bestSceneSegmentMatches2);
#else
					// recognition.ICP(PCLICP, PCLICPVariants::Point_to_plane, recognition.bestSceneSegmentMatches2);
					recognition.ICP_refined(PCLICP, PCLICPVariants::Point_to_plane, recognition.bestSceneSegmentMatches2);
#endif

					// CUDA MATRICES PREPARATION - TEST
					// recognition.PrepareCUDACTIMatchingMatrices(&W, &VA, &V, &Ds, &Rs, &ts, true);
					// recognition.PrepareCUDACTIMatchingOfflineMatrices(&Dm, &Rm_inv, &tm_inv, true);

					// Create columnWise matrices
					// recognition.PrepareCUDACTIMatchingMatrices_ColumnWise(&W, &VA, &V, &Ds, &Rs, &ts, true);
					// recognition.PrepareCUDACTIMatchingOfflineMatrices_ColumnWise(&Dm, &Rm_inv, &tm_inv, true);

					// recognition.SaveMatchesMatrix();
					// END

					// Timer ICP END
					QueryPerformanceCounter((LARGE_INTEGER *)&CNTREvaluation3END);

					// TEST ECVV Vidovic
					// recognition.ICP_refined(PCLICP, PCLICPVariants::Point_to_point, recognition.bestSceneSegmentMatches2);
					// END
#endif // #ifndef RVLPSGM_CUDA_ICP
	   // Timer HP START
					QueryPerformanceCounter((LARGE_INTEGER *)&CNTRHypothesisPruningSTART);

					recognition.HypothesisEvaluation(recognition.bestSceneSegmentMatches2, true);

					if (recognition.bVisualizeHypothesisEvaluationLevel2)
						recognition.VisualizeHypotheses(recognition.bestSceneSegmentMatches2, true);

					// Colision check
					recognition.noCollisionHypotheses.clear();
					recognition.transparentHypotheses.clear();
					recognition.envelopmentColisionHypotheses.clear();
					recognition.GetHypothesesCollisionConsensus(&recognition.noCollisionHypotheses, &recognition.bestSceneSegmentMatches2, 10);

					// Timer HP END
					QueryPerformanceCounter((LARGE_INTEGER *)&CNTRHypothesisPruningEND);

					// Get transparency and collision consensus
					if (flags & RVLRECOGNITION_DEMO_FLAG_3D_VISUALIZATION)
						recognition.GetTransparencyAndCollisionConsensus(&visualizer, bVerbose);
					else
						recognition.GetTransparencyAndCollisionConsensus(NULL, bVerbose);

#ifdef RVLRECOGNITION_DEMO_EVALUATION
					// Evaluate consesus matches
					float precision, recall;
					recognition.EvaluateConsensusMatches(precision, recall, true);

#ifdef RVLPSGM_RMSE_CALCULATION
					// Calculate RMSE
					recognition.RMSE_Consensus(fpRMSE, true);
#endif
#endif
#else // #ifndef RVLVERSION_171125
					recognition.ICP(PCLICP, PCLICPVariants::Point_to_plane, recognition.bestSceneSegmentMatches);

#ifdef RVLPSGM_TRANSPARENCY_AND_COLLISION
					// Transparency check
					// recognition.CreateScoreMatchMatrixICP();
					recognition.CreateScoreMatchMatrixICP_TMP();
					recognition.FilterHypothesesUsingTransparency(0.15, 10, true);
					recognition.CreateScoreMatchMatrixICP_TMP(); // because of sorting - TEST

					// Get scene consistency
					// recognition.GetSceneConsistancy(&recognition.scoreMatchMatrixICP, 0.1, 30, 20, true);

					// Colision check
					recognition.noCollisionHypotheses.clear();
					recognition.GetHypothesesCollisionConsensus(&recognition.noCollisionHypotheses, &recognition.scoreMatchMatrixICP, 10);

					// Get transparency and collision consensus
					recognition.GetTransparencyAndCollisionConsensus(&visualizer);
#endif

					// Evaluate consesus matches
					float precision, recall;
					recognition.EvaluateConsensusMatches(precision, recall, true);

					// recognition.createVersionTestFile();
					recognition.checkVersionTestFile();

#ifdef RVLPSGM_DETERMINE_THRESHOLDS
					// determine thresholds for SHAPE_INSTANCE_DETECTION
					recognition.DetermineThresholds();
#endif
#endif // #ifndef RVLVERSION_171125

					// #ifdef RVLVERSION_171125
					//				//evaluate ICP
					//				recognition.EvaluateMatchesByScore(fpHypothesisEvaluation, fpLog, fpPoseError, fpnotFirstInfo, fpnotFirstPoseErr, 10, true);
					// #endif

#ifdef NEVER
					// Load models without decimation (used for calculatin RMSE)
					recognition.LoadModelMeshDB(modelSequenceFileName, &recognition.vtkRMSEModelDB, false);

					// Calculate RMSE
					recognition.RMSE(fpRMSE, false);
#endif
				}
#else // #ifndef RVLPSGM_ICP
	  // recognition.EvaluateMatchesByScore(fpHypothesisEvaluation, fpLog, fpPoseError, fpnotFirstInfo, fpnotFirstPoseErr, 7);
#endif // #ifndef RVLPSGM_ICP
	   // recognition.AddModelsToVisualizer(&visualizer, true, PCLICP, PCLICPVariants::Point_to_plane, NULL/*&kdtree*/);
	   // QueryPerformanceCounter((LARGE_INTEGER *)&ctr2_);
	   // QueryPerformanceFrequency((LARGE_INTEGER *)&freq_);

#ifdef RVLVN_TIME_MESUREMENT
				QueryPerformanceFrequency((LARGE_INTEGER *)&frequency);

				float timeICP = (CNTREvaluation3END.QuadPart - CNTREvaluation3START.QuadPart) * 1000.0 / frequency.QuadPart;
				float timeHypothesisPruning = (CNTRHypothesisPruningEND.QuadPart - CNTRHypothesisPruningSTART.QuadPart) * 1000.0 / frequency.QuadPart;

				std::cout << "**********************************************" << std::endl;
				std::cout << "Vremena izvodjenja: " << std::endl;
				std::cout << "Segmentation: " << recognition.timeSegmentation << " ms" << std::endl;
				std::cout << "Descriptor generation: " << recognition.timeDescriptorGeneration << " ms" << std::endl;
				std::cout << "Descriptor matching: " << recognition.timeDescriptorMatchingAndE1 << " ms" << std::endl;
				std::cout << "Tangent alignment: " << recognition.timeTangentAlignmentAndE2 << " ms" << std::endl;
				std::cout << "ICP: " << timeICP << " ms" << std::endl;
				std::cout << "Hypothesis pruning: " << timeHypothesisPruning << " ms" << std::endl;
				std::cout << "----------------------------------------------" << std::endl;
				std::cout << "Ukupno vrijeme: " << recognition.timeSegmentation + recognition.timeDescriptorGeneration + recognition.timeDescriptorMatchingAndE1 + recognition.timeTangentAlignmentAndE2 + timeICP + timeHypothesisPruning << " ms" << std::endl;
				std::cout << "**********************************************" << std::endl;

				fprintf(fpTime, "%f\t%f\t%f\t%f\t%f\t%f\n", recognition.timeSegmentation, recognition.timeDescriptorGeneration, recognition.timeDescriptorMatchingAndE1, recognition.timeTangentAlignmentAndE2, timeICP, timeHypothesisPruning);
#endif

				// Save the poses of the first detected object into a file (for Danieli_robot project and ECMR 2019).

				fpForegroundObject = fopen((std::string(ResultsFolder) + "\\detected_object.m").data(), "a");

				recognition.SaveObjectPose(fpForegroundObject);

				fclose(fpForegroundObject);

				//

				if (flags & RVLRECOGNITION_DEMO_FLAG_3D_VISUALIZATION)
					visualizer.Run();

				if (bSceneBrowser)
				{
					printf("0 - repeate scene; 1 - next scene; 2 - select scene; 3 - exit\n");

					scanf("%d", &command);

					if (command == 3)
						break;
				}
			} // for every scene

			// Only for results creation
			printf("Total number of segments in the experiment is: %d\n", recognition.nExperimentSegments);
			// END

			// CUDA MATRICES PREPARATION - TEST
			delete[] W;
			delete[] VA;
			delete[] Ds;
			delete[] Rs;
			delete[] ts;
			delete[] Dm;
			delete[] Rm_inv;
			delete[] tm_inv;

			RVL_DELETE_ARRAY(CTIFileName);

			RVL_DELETE_ARRAY(recognition.pTimer);

			RVL_DELETE_ARRAY(recognition.segmentGT.Element);

			fclose(fpHypothesisEvaluation);
			fclose(fpLog);
			fclose(fpPoseError);
			fclose(fpRMSE);
			fclose(fpTime);

			// END Vidovic
		} // if (recognition.mode == RVLRECOGNITION_MODE_RECOGNITION)
		else if (recognition.mode == RVLRECOGNITION_MODE_PSGM_CREATE_CTIS)
		{
			Mesh mesh;

			FileSequenceLoader sceneSequence;

			sceneSequence.Init(sceneSequenceFileName);

			recognition.pECCVGT->Init(sceneSequence, GTFolder, modelsInDB);

			char *clusterNormalDistributionFileName = NULL;

			int iScene = 0;

			char filePath[200];
			FILE *fpClusterNormalDistribution;

			// char filePath[200];

			///

			int nM = 35, nSM = 3;

			while (sceneSequence.GetNextPath(filePath))
			{

				printf("Scene %s...\n", filePath);

				// mesh.LoadPolyDataFromPLY(filePath);
#ifdef RVLPCL
				LoadMesh(&meshBuilder, filePath, &mesh, false);
#endif

				recognition.SetSceneFileName(filePath);

				// Alokacija prostora za matcheve - TEMP
				//  List of matches
				recognition.SMatch = new RECOG::PSGM_::SegmentMatch[recognition.nDominantClusters * nM * nSM]; // Petra

				// Sorted matches
				recognition.sortedMatches = new SortIndex<float>[recognition.nDominantClusters * nM * nSM]; // Petra

				recognition.Interpret(&mesh, iScene);
				// recognition.InterpreteCTIS(&mesh);

				// recognition.LoadCTI("D:\\ARP3D\\ECCV_dataset\\pcd_files\\frame_20111220T111153.549117.cti");

				/*TEST KRETANJA KROZ CTI
				int nCTI = recognition.CTI.n;
				recognition.CTI.Element[0];

				recognition.CTI.Element[0].modelInstance.Element[25].d;

				RECOG::PSGM_::ModelInstance *pCTI;
				pCTI = recognition.CTI.Element;

				//pCTI->modelInstance.Element[0]->d

				RECOG::PSGM_::ModelInstanceElement *pMIE;

				pMIE = pCTI->modelInstance.Element;

				pMIE->d;
				*/
				RVLCopyString(filePath, &clusterNormalDistributionFileName);

				sprintf(RVLGETFILEEXTENSION(clusterNormalDistributionFileName), "seg");

				fpClusterNormalDistribution = fopen(clusterNormalDistributionFileName, "w");

				recognition.WriteClusterNormalDistribution(fpClusterNormalDistribution);

				fclose(fpClusterNormalDistribution);

				printf("Scene %s...finished!\n\n", filePath);

				iScene++;

				// Visualization
				// surfels.NodeColors(SelectionColor);
				////visualizer.renderer->RemoveAllViewProps();
				// recognition.InitDisplay(&visualizer, &mesh, SelectionColor);
				// recognition.Display();
				// visualizer.Run();
			}

			RVL_DELETE_ARRAY(clusterNormalDistributionFileName);

			// Visualization
			surfels.NodeColors(SelectionColor);
			recognition.InitDisplay(&visualizer, &mesh, SelectionColor);
			recognition.Display();
			visualizer.Run();
		}
#else
		printf("PCL required!\n");
#endif
	} // if (method == RVLRECOGNITION_METHOD_PSGM)
	else if (method == RVLRECOGNITION_METHOD_VN)
	{

		// Parameters

		// float voxelSize = 5.0f;
		// int sampleVoxelDistance = 1;
		// float eps = 2.0f;
		// float resolution = 1.0f;
		// float voxelSize = 0.02f;
		// int sampleVoxelDistance = 1;
		// float eps = 0.01f;
		float resolution = 0.01f;

		Mesh mesh;

		VNClassifier classifier;

		// Gripper gripper;
		// gripper.CreateGripper();

		classifier.pMem0 = &mem0;
		classifier.pMem = &mem;
#ifdef RVLPCL
		classifier.vpMeshBuilder = &meshBuilder;
		classifier.LoadMesh = LoadMesh;
		classifier.ICPFunction = PCLICP;
		classifier.ICPVariant = PCLICPVariants::Point_to_plane;
#endif
		classifier.pSurfels = &surfels;
		classifier.pSurfelDetector = &surfelDetector;

		classifier.visualizationData.resolution = resolution;
		classifier.visualizationData.SDFSurfaceValue = SDFSurfaceValue;

		if (flags & (RVLRECOGNITION_DEMO_FLAG_VISUALIZE_VN_MODEL | RVLRECOGNITION_DEMO_FLAG_VISUALIZE_VN_INSTANCE) || bCreateVisibleSurfaceMesh)
			classifier.bLoadCTIDataBase = false;

		classifier.Create(cfgFileName);

#ifdef RVLPCL
		classifier.camera.fu = meshBuilder.camera.depthFu;
		classifier.camera.fv = meshBuilder.camera.depthFv;
		classifier.camera.uc = meshBuilder.camera.depthUc;
		classifier.camera.vc = meshBuilder.camera.depthVc;
		classifier.camera.w = meshBuilder.camera.w;
		classifier.camera.h = meshBuilder.camera.h;
#endif

		classifier.alignment.resultsFolder = ResultsFolder;

		std::string resultsFolderName = std::string(classifier.alignment.resultsFolder);
		FILE *fpVNHyp = fopen((resultsFolderName + "\\VNClassify.txt").data(), "w");

		FILE *fpVNScore = fopen((resultsFolderName + "\\VNScore.txt").data(), "w");
		fclose(fpVNScore);

		classifier.alignment.modelsInDataBase = classifier.modelsInDataBase;

		// classifier.alignment.LoadModelMeshDB(modelSequenceFileName, &classifier.alignment.vtkModelDB, false, 0.4);

		// classifier.alignment.debug1 = 7;
		classifier.alignment.wTransparency1 = 0.6;

		classifier.resultsFolder = ResultsFolder;

		if (classifier.pTimer == NULL)
			classifier.pTimer = new CRVLTimer;

		classifier.pTimer->Start();

		// Model visualization.

		if (flags & RVLRECOGNITION_DEMO_FLAG_VISUALIZE_VN_MODEL)
		{
			int iMetaModel = classifier.classArray.Element[iClass].iMetaModel;

			VN *pModel = classifier.models[iMetaModel];

			// Box<float> box;

			// box = pModel->boundingBox;

			// ExpandBox<float>(&box, 2.0f * resolution);

			// pModel->Display(&visualizer, box, resolution, NULL, NULL, SDFSurfaceValue);

			pModel->Display(&visualizer, 0.01f, NULL, NULL, SDFSurfaceValue);

			visualizer.Run();
		}
		else if (flags & RVLRECOGNITION_DEMO_FLAG_VISUALIZE_VN_INSTANCE)
		{
			int command = 1;

			while (command)
			{
				FILE *fp = fopen(instanceFileName, "r");

				float *d = NULL;
				bool *bd = NULL;
				float *q = NULL;

				int iMetaModel;

				// int iModel;

				// classifier.LoadDescriptor(fp, d, bd, iModel, iMetaModel);

				// fclose(fp);

				int iClass;

				classifier.LoadLatentVector(fp, q, iClass);

				fclose(fp);

				RECOG::ClassData *pClass = classifier.classArray.Element + iClass;

				d = new float[pClass->M.w];

				int i, j;
				float *a;

				RVLMULMXTVECT(pClass->M.Element, q, pClass->M.h, pClass->M.w, d, i, j, a);

				iMetaModel = pClass->iMetaModel;

				visualizer.renderer->RemoveAllViewProps();

				VN *pModel = classifier.models[iMetaModel];

				pModel->Display(&visualizer, 0.01f, d, bd, classifier.visualizationData.SDFSurfaceValue);

				visualizer.Run();

				RVL_DELETE_ARRAY(d);
				RVL_DELETE_ARRAY(bd);
				RVL_DELETE_ARRAY(q);

				printf("0 - exit; 1 - one more. ");

				scanf("%d", &command);
			}
		}
		else if (classifier.mode == RVLRECOGNITION_MODE_TRAINING)
		{
			// classifier.LearnPrimitives(primitiveFileName);

			classifier.visualizationData.pVisualizer = &visualizer;

			classifier.Learn(modelSequenceFileName, iClass, iMetamodel, &visualizer); // Vidovic
		}
		else if (classifier.mode == RVLRECOGNITION_MODE_RECOGNITION)
		{
			// FileSequenceLoader dbLoader;

			// dbLoader.Init(classifier.modelsInDataBase);

			// char refModelFileName[200];

			// dbLoader.GetFilePath(classifier.classArray.Element[iClass].iRefInstance, refModelFileName);

			// int iMetaModel = classifier.classArray.Element[iClass].iMetaModel;

			// VN *pModel = classifier.models[iMetaModel];

			// RVL_DELETE_ARRAY(classifier.refModel.d);
			// RVL_DELETE_ARRAY(classifier.refModel.bd);

			// int iModel_, iMetaModel_;

			// char *refModelDescriptorFileName = RVLCreateFileName(refModelFileName, ".ply", -1, ".vnd");

			// FILE *fpVNDescriptor = fopen(refModelDescriptorFileName, "r");

			// delete[] refModelDescriptorFileName;

			// classifier.LoadDescriptor(fpVNDescriptor, classifier.refModel.d, classifier.refModel.bd, iModel_, iMetaModel_);

			// fclose(fpVNDescriptor);

			// if (classifier.problem == RVLRECOGNITION_PROBLEM_PART_SEGMENTATION)
			//	classifier.pObjects = new SURFEL::ObjectGraph;

#ifdef RVLPCL
			PCLNeighborhoodTool<pcl::PointXYZINormal, pcl::PointCloud<pcl::PointXYZINormal>::Ptr, Point> neighborhoodTool;
			classifier.pNeighborhoodTool = &neighborhoodTool;

			pcl::PointCloud<pcl::PointXYZINormal>::Ptr nieghborhoodToolPC(new pcl::PointCloud<pcl::PointXYZINormal>);
			neighborhoodTool.SetPointCloud(nieghborhoodToolPC);
#endif

			classifier.visualizationData.pVisualizer = &visualizer;

			FileSequenceLoader sceneSequence;

			sceneSequence.Init(sceneSequenceFileName);

			if (classifier.pShapeInstanceDetection)
			{
				classifier.pShapeInstanceDetection->camera = classifier.camera;

				if (bGT)
				{
					classifier.pShapeInstanceDetection->pECCVGT->Init(sceneSequence, GTFolder, modelsInDB, classifier.dataSet);

					classifier.pShapeInstanceDetection->pECCVGT->LoadRotInvarianceInfo(rotInvarianceInfoFileName);
				}
			}

			char filePath[200];
			char fileName[200];

			FILE *fCorrespondences = fopen("C:\\RVL\\ExpRez\\Correspondences.txt", "w");
			fclose(fCorrespondences);
			FILE *fpDescriptors = fopen("C:\\RVL\\ExpRez\\descriptors.txt", "w");
			fclose(fpDescriptors);
			FILE *fmIoU = fopen("C:\\RVL\\ExpRez\\mIoU.txt", "w");
			fclose(fmIoU);
			FILE *fpDescriptorsMerged = fopen("C:\\RVL\\ExpRez\\descriptorsMerged.txt", "w");
			fclose(fpDescriptorsMerged);
			FILE *fpNoHypotheses = fopen((std::string(classifier.resultsFolder) + "\\noHyp.dat").data(), "w");
			fclose(fpNoHypotheses);
			FILE *fpRMSE = fopen((std::string(classifier.resultsFolder) + "\\RMSE.dat").data(), "w");
			fclose(fpRMSE);
			classifier.sceneID = 0;
			classifier.iSelectedClass = iClass;

			// WILLOW AND CHALLENGE DATASET GT FILES TRANSFORM
			// TransformWillowAndChallengeGT2ECCVFormat("F:\\Projekti\\ARP3D\\Auxiliary\\willow_and_challenge_dataset\\annotations\\willow\\sceneGTSequence.txt");

			// ICL DATASET GT FILES TRANSFORM
			// TransformCorrectedICLGT2ECCVFormat("F:\\Projekti\\ARP3D\\Auxiliary\\ICLDataset\\annotation_new_models\\juice_carton\\sceneGTSequence.txt");
			// TransformICLGT2ECCVFormat("F:\\Projekti\\ARP3D\\Auxiliary\\ICLDataset\\Annotation\\coffee_cup\\sceneGTSequence.txt");

			QList<VNInstance> *pSceneVNList = &classifier.sceneVNList;
			RVLQLIST_INIT(pSceneVNList);

			FileSequenceLoader dbLoader;
			int ID;

			if (classifier.bLoadSceneVNInstances)
			{
				dbLoader.Init(classifier.scenesInDataBase);
				classifier.LoadVNInstances(&classifier, classifier.SceneVNInstanceListFileName, pSceneVNList, &(classifier.sceneVNArray));
			}

			while (sceneSequence.GetNext(filePath, fileName))
			{

				if (classifier.bLoadSceneVNInstances)
				{

					RVL::RECOG::ModelExistInDB(filePath, dbLoader, &ID);
				}

				printf("Scene: %s:\n", filePath);
				classifier.alignment.SetSceneFileName(filePath);
				classifier.SetSceneFileName(filePath);

				// Load mesh.

#ifdef RVLPCL
				LoadMesh(&meshBuilder, filePath, &mesh, false);
#endif

				// Load labels.

				// char *labelFileName = RVLCreateFileName(filePath, ".ply", -1, "_.seg");

				// mesh.LoadLabels(labelFileName);

				// delete[] labelFileName;

				// Reset memory.

				mem.Clear();

				if (classifier.problem == RVLRECOGNITION_PROBLEM_CLASSIFICATION || classifier.problem == RVLRECOGNITION_PROBLEM_SHAPE_INSTANCE_DETECTION)
				{
					// Classification.

					classifier.Interpret(&mesh);

					// Save the segmentation results to a file.

					// cv::Mat objectMask(mesh.height, mesh.width, CV_8UC1);

					// classifier.pObjects->ObjectMapMask(&objectMask);

					// cv::imshow("Object mask", objectMask);

					// cv::waitKey();

					// char *objectMapFileName = RVLCreateFileName(filePath, ".ply", -1, ".objmap.png");

					// cv::imwrite(objectMapFileName, objectMask);

					// delete[] objectMapFileName;
				}
				else if (classifier.problem == RVLRECOGNITION_PROBLEM_PART_SEGMENTATION)
				{
					FILE *fCorrespondences = fopen("C:\\RVL\\ExpRez\\Correspondences.txt", "a");
					fprintf(fCorrespondences, "Scene: %s:\n", filePath);
					fclose(fCorrespondences);

					if (classifier.bLoadSceneVNInstances)
						classifier.SegmentToParts(&mesh, ID);
					else
					{
						classifier.SegmentToParts(&mesh);

						if (classifier.bGenerateSceneVNInstances)
							dbLoader.AddModel(classifier.sceneID, filePath, fileName);
					}
				}
				classifier.sceneID++;
			}

			if (classifier.bGenerateSceneVNInstances && classifier.bSaveGeneratedVNInstances)
			{
				classifier.SaveVNInstances(&classifier.sceneVNList, &classifier);
				RECOG::SaveModelID(dbLoader, classifier.scenesInDataBase);
			}

			FILE *fpInterpretation = fopen((std::string(classifier.resultsFolder) + "\\interpretation.txt").data(), "a");

			float precision = (float)classifier.nTP / ((float)classifier.nTP + (float)classifier.nFP);
			float recall = (float)classifier.nTP / ((float)classifier.nTP + (float)classifier.nFN);

			fprintf(fpInterpretation, "\nTP: %d\nFP: %d\nFN: %d\nprecision: %f\nrecall: %f\n\n", classifier.nTP, classifier.nFP, classifier.nFN, precision, recall);

			fclose(fpInterpretation);

			FILE *fpFN = fopen((std::string(classifier.resultsFolder) + "\\FN.dat").data(), "w");
			fprintf(fpFN, "%d", classifier.nFN);
			fclose(fpFN);

			// if (classifier.problem == RVLRECOGNITION_PROBLEM_PART_SEGMENTATION)
			//	delete classifier.pObjects;
		}
		fclose(fpVNHyp);
	} // if (method == RVLRECOGNITION_METHOD_VN)
	  // T-Graph Matching.
#ifdef RVLRECOGNITION_DEMO_RVLTG
	else if (method == RVLRECOGNITION_METHOD_TG)
	{
		TGraph MTG, QTG;

		RECOG::TGRAPH::TGraphComPar TGraphComPar;

		TGraphComPar.pMem = &mem;
		TGraphComPar.pSurfels = &surfels;
		TGraphComPar.pSurfelDetector = &surfelDetector;
		TGraphComPar.tetrahedronAggregationMethod = RVLTGRAPH_TETRAHEDRON_AGGREGATION_METHOD_CLUSTERS;

		FILE *fpRnd = fopen("..\\pseudorandom1000000.dat", "rb");
		TGraphComPar.iRnd.n = 1000000;
		TGraphComPar.iRnd.Element = new int[TGraphComPar.iRnd.n];
		fread(TGraphComPar.iRnd.Element, sizeof(int), TGraphComPar.iRnd.n, fpRnd);
		fclose(fpRnd);

		MTG.resultsFolder = std::string(ResultsFolder);

		FILE *fp = fopen((MTG.resultsFolder + "\\A.txt").data(), "w");

		float *a;

		for (int i = 0; i < 66; i++)
		{
			a = MTG.A + 3 * i;

			fprintf(fp, "%f %f %f\n", a[0], a[1], a[2]);
		}

		fclose(fp);

		FileSequenceLoader modelsLoader;
		char modelFilePath[200];
		char modelFileName[200];

		modelsLoader.Init(modelSequenceFileName);

		modelsLoader.GetNext(modelFilePath, modelFileName);

#ifdef RVLPCL
		printf("\Creating TetGraph from model %s ...", modelFileName);

		Mesh MMesh, QMesh;

		if (LoadMesh(&meshBuilder, modelFilePath, &MMesh, false, NULL, NULL))
		{
			MTG.Create(&MMesh, modelFilePath, &TGraphComPar);

			printf("completed.\n");

			double *z = new double[3 * MTG.anchors.n];
			while (modelsLoader.GetNext(modelFilePath, modelFileName))
			{
				if (LoadMesh(&meshBuilder, modelFilePath, &QMesh, false, NULL, NULL))
				{
					printf("\nProcessing query model %s... ", modelFileName);
					QTG.AnchorCandidates(&QMesh);
					double *zInit;
					MTG.TangentDomainAssociation(&QTG, zInit);
					// MTG.AnchorAssociation(&QTG, zInit);
					// memset(z, 0, 3 * MTG.anchors.n * sizeof(double));
					MTG.Fit(&QMesh, zInit, 0.0001f, 0.10f, 45.0f, 5, z, false);
					printf("completed\n");
					Mesh deformedMMesh;
					MTG.CreateDeformedMesh(z, &deformedMMesh);
					MTG.pVisualizationData->pVisualizer->renderer->RemoveAllViewProps();
					MTG.pVisualizationData->pVisualizer->SetMesh(deformedMMesh.pPolygonData);
					MTG.pVisualizationData->pVisualizer->Run();
					MTG.pVisualizationData->pVisualizer->renderer->RemoveAllViewProps();
					delete[] zInit;
				}
				else
					printf("Cannot open %s!\n", modelFileName);
			}
			delete[] z;
		}
		else
			printf("\nCannot open %s!\n", modelFileName);
#else
		printf("ERROR: Mesh builder is required!\n");
#endif

		delete[] TGraphComPar.iRnd.Element;
	}
#endif
	// Drawer and Door Detector.
	else if (method == RVLRECOGNITION_METHOD_DDD)
	{
		printf("Creating DDDetector...\n");
		DDDetector detector;
		detector.pMem0 = &mem0;
		detector.pMem = &mem;
		detector.Create(cfgFileName);
		detector.camera = camera;
		cv::Mat *camsMatrix = NULL;
		cv::Mat *camsDist = NULL;
		if (detector.test == RVLDDD_TEST_3DTO2DFIT)
		{
			camsMatrix = new cv::Mat[2];
			camsDist = new cv::Mat[2];
			detector.loadBaslerCamerasParams(cameraParamsFileName, camsMatrix, camsDist); // loads params into detector.cameras
		}
		cv::Mat distCoeffs;
		detector.pSurfels = &surfels;
		detector.pSurfelDetector = &surfelDetector;
		detector.InitVisualizer(&visualizer);
		detector.resultsFolder = ResultsFolder;
		Mesh *pMesh;
		Array<Mesh> meshSeq;
		meshSeq.Element = NULL;
		std::vector<cv::Mat> RGBSeq;
		Array<RECOG::DDD::Transformation> sceneTransformations;
		sceneTransformations.Element = NULL;
		RECOG::DDD::Transformation *pSceneTransformation;

		// Load model meshes.

		std::vector<std::string> modelFileNames;
		if (detector.mode == RVLRECOGNITION_MODE_TRAINING)
		{
			Array<Mesh> models;
			models.Element = NULL;
			printf("Loading models...\n");
			FileSequenceLoader modelLoader;
			modelLoader.Init(modelSequenceFileName);
			char modelFilePath[200];
			char modelFileName[200];
			models.Element = new Mesh[modelLoader.nFileNames];
			pMesh = models.Element;
			while (modelLoader.GetNext(modelFilePath, modelFileName))
			{
				printf("Loading model %s\n", modelFileName);
#ifdef RVLPCL
				if (bMeshBuilder)
					LoadMesh(&meshBuilder, modelFilePath, pMesh++, false);
				else
#endif
				{
					pMesh->LoadFromPLY(modelFilePath, maxMeshTriangleEdgeLen, true, &camera);
					pMesh++;
				}
				modelFileNames.push_back(modelFilePath);
			}
			printf("completed.\n");
			models.n = pMesh - models.Element;

			// Create Models from meshes.

			if (detector.test == RVLDDD_TEST_DETECT_RECTSTRUCT || detector.test == RVLDDD_TEST_RECOGNIZE_RECTSTRUCT)
			{
				RECOG::DDD::RectStruct rectStruct;
				float verticalAxis[3];
				RVLSET3VECTOR(verticalAxis, 0.0f, -1.0f, 0.0f);
				Array<RECOG::DDD::EdgeLineSegment> edgeLineSegments;
				int *edgeLineSegmentPixIdxMem;
				int *edgeLineSegmentMap;
				for (int iModel = 0; iModel < models.n; iModel++)
				{
					pMesh = models.Element + iModel;
					if (detector.RectangularStructures(pMesh, &rectStruct))
					{
						// std::string modelInFileName = modelFileNames[iModel];
						// std::string modelOutFileName = modelInFileName.substr(0, modelInFileName.rfind('.') + 1) + "dat";
						// detector.SaveRectangularStructure(modelOutFileName, &rectStruct);
						cv::Mat BGR;
						pMesh->GetBGR(BGR);

						// SLIC test.

						// IplImage BGR_ = cvIplImage(BGR);
						// IplImage* lab_image = cvCloneImage(&BGR_);
						// cvCvtColor(&BGR_, lab_image, CV_BGR2Lab);
						// int w = BGR.cols, h = BGR.rows;
						// int nr_superpixels = 400;
						// int nc = 10;
						// double step = sqrt((w * h) / (double)nr_superpixels);
						// Slic slic;
						// slic.generate_superpixels(lab_image, step, nc);
						// slic.create_connectivity(lab_image);
						// slic.display_contours(&BGR_, CV_RGB(255, 0, 0));
						// cvShowImage("result", &BGR_);
						// cvWaitKey(0);

						//

						cv::Mat edges, sobelx, sobely;
						// detector.DetectRGBEdges2(BGR, edges, sobelx, sobely, edgeLineSegments, edgeLineSegmentPixIdxMem, edgeLineSegmentMap);
						detector.DetectRGBEdgeLineSegments(BGR, 20, 100, 50, edges, sobelx, sobely, edgeLineSegments, edgeLineSegmentPixIdxMem, edgeLineSegmentMap);

						// Display edges on disparity image.

						// Array2D<short> depthImg;
						// pMesh->GetDepth(depthImg);
						// cv::Mat depthDisplayImg(depthImg.h, depthImg.w, CV_8UC3);
						// DisplayDisparityMap(depthImg, (uchar*)(depthDisplayImg.data), false, RVLRGB_DEPTH_FORMAT_1MM);
						// int nPix = depthImg.w * depthImg.h;
						// uchar* visPix = depthDisplayImg.data;
						// for (int iPix = 0; iPix < nPix; iPix++, visPix += 3)
						//	if (edges.data[iPix])
						//		RVLSET3VECTOR(visPix, 0, 255, 0)
						// cv::imshow("depth and edges", depthDisplayImg);
						// cv::waitKey();
						// delete[] depthImg.Element;

						// Orthogonal views.

						std::vector<cv::Mat> orthogonalViews;
						int nOrthogonalViews = 3;
						Array<Array2D<uchar>> masks;
						Array<RECOG::DDD::Line2D> *orthogonalViewLines;
						Pose3D *poseFC;
                        detector.DDOrthogonalView(&rectStruct, edgeLineSegments, verticalAxis, nOrthogonalViews, poseFC, orthogonalViews, masks, orthogonalViewLines, true, pMesh);
						delete[] edgeLineSegments.Element;
						delete[] edgeLineSegmentPixIdxMem;
						delete[] edgeLineSegmentMap;

						// Detect doors and drawers on orthogonal views.

						Array<Rect<int>> *DDRects = new Array<Rect<int>>[nOrthogonalViews];
						for (int iView = 0; iView < nOrthogonalViews; iView++)
							detector.Detect3(masks.Element[iView], orthogonalViewLines + iView, DDRects + iView);

						// Display doors and drawers.

						cv::Mat displayImg = BGR.clone();
						detector.Display(displayImg, poseFC, nOrthogonalViews, DDRects);
                        RECOG::DDD::Detect3CallBackFuncData displayCallBackFuncData;
                        displayCallBackFuncData.vpDetector = &detector;
                        displayCallBackFuncData.nOrthogonalViews = nOrthogonalViews;
                        displayCallBackFuncData.orthogonalViewLines = orthogonalViewLines;
                        displayCallBackFuncData.DDRects = DDRects;
                        cv::setMouseCallback("Doors and Drawers", RECOG::DDD::Detect3CallBackFunc, &displayCallBackFuncData);
						cv::imshow("Doors and Drawers", displayImg);
						cv::waitKey();

						//

						for (int iView = 0; iView < nOrthogonalViews; iView++)
						{
							delete[] orthogonalViewLines[iView].Element;
							delete[] DDRects[iView].Element;
						}
						delete[] orthogonalViewLines;
						delete[] DDRects;
						delete[] poseFC;

						// Display edges on orthogonal views.

						// for (int iView = 0; iView < nOrthogonalViews; iView++)
						//{
						//	int w = orthogonalViews[iView].cols;
						//	int h = orthogonalViews[iView].rows;
						//	cv::Mat transfImg;
						//	transfImg.create(orthogonalViews[iView].size(), CV_8UC1);
						//	RECOG::DDD::MapImageC1(edges, orthogonalViews[iView], transfImg);
						//	cv::Mat displayImg;
						//	displayImg.create(orthogonalViews[iView].size(), CV_8UC3);
						//	uchar* pSrcPix = transfImg.data;
						//	uchar* pTgtPix = displayImg.data;
						//	uchar* pMaskPix = masks.Element[iView].Element;
						//	for (int iPix = 0; iPix < w * h; iPix++, pSrcPix++, pTgtPix += 3, pMaskPix++)
						//	{
						//		if (*pMaskPix)
						//		{
						//			if(*pSrcPix)
						//				RVLSET3VECTOR(pTgtPix, 255, 255, 255)
						//			else
						//				RVLSET3VECTOR(pTgtPix, 0, 0, 0)
						//		}
						//		else
						//			RVLSET3VECTOR(pTgtPix, 255, 0, 0)
						//	}
						//	cv::imshow("orthogonal view edges", displayImg);
						//	cv::waitKey();
						// }

						//

						for (int i = 0; i < nOrthogonalViews; i++)
							delete[] masks.Element[i].Element;
						delete[] masks.Element;
					}
					delete[] rectStruct.rects.Element;
				}
			}
			else
				detector.CreateModels(models, modelFileNames);

			RVL_DELETE_ARRAY(models.Element);
		}

		// Load image sequence.

		else if (detector.mode == RVLRECOGNITION_MODE_RECOGNITION)
		{
			// Loading image sequence.
			char imgFilePath[200];
			char imgFileName[200];
			std::string sceneSequenceTxt = std::string(RVLFILEPATH_SEPARATOR_) + "sceneSequence.txt";

			std::vector<std::string> sequenceFileNames;
			// Loading sequence file names from sceneSeqSeqFileName and storing them in sequenceFileNames.
			if (sceneSequenceSequenceFileName != NULL)
			{
				FileSequenceLoader sequenceSequenceLoader;
				if (!detector.bLoadMovingPartHypothesesFromFile || !detector.bLoadDDHypothesesFromFile || detector.GetVisualizeDoorHypotheses())
				{
					printf("Loading sequence of image sequences...\n");
					sequenceSequenceLoader.Init(sceneSequenceSequenceFileName);
				}

				while (sequenceSequenceLoader.GetNext(imgFilePath, imgFileName))
				{
					// strcat(imgFilePath, sceneSequenceTxt.c_str());
					sequenceFileNames.push_back(imgFilePath);
				}
			}
			else
			{
				sequenceFileNames.push_back(sceneSequenceFileName);
			}

			RVL_DELETE_ARRAY(sceneSequenceFileName);

			// // LOOP OVER MORE SEQUENCES
			// cv::Mat pRGBDisplay = cv::imread("/home/RVLuser/rvl-linux/data/COSPER/AO/Exp-IRI-221121/images/scene_wardrobe_human/pose_00_00_wardrobe_human_left_door_2023-03-29-15-17-21/rgb/0000.png");
			// std::string csvFileName = "/home/RVLuser/rvl-linux/data/COSPER/AO/Exp-IRI-221121/images/scene_wardrobe_human/pose_00_00_wardrobe_human_left_door_2023-03-29-15-17-21/bbox_gt/gt.csv";
			// std::vector<std::vector<std::string>> csvContent;
			// std::vector<std::vector<PSD::Point2D>> allGTPoints;
			// bool x = detector.ParseCSV(csvFileName, true, csvContent);
			// detector.GetGTPointsFromCSV(csvContent, allGTPoints);

			// for (int gtPt = 0; gtPt < allGTPoints.size(); gtPt++)
			// {
			// 	std::vector<PSD::Point2D> gtPts = allGTPoints[gtPt];
			// 	for (int iPt = 0; iPt < 4; iPt++)
			// 	{
			// 		cv::line(pRGBDisplay,
			// 				 cv::Point(gtPts[iPt].P[0], gtPts[iPt].P[1]),
			// 				 cv::Point(gtPts[(iPt + 1) % gtPts.size()].P[0], gtPts[(iPt + 1) % gtPts.size()].P[1]),
			// 				 cv::Scalar(0, 255, 0), 2);
			// 	}
			// }
			// cv::imshow("aa", pRGBDisplay);
			// cv::waitKey();

			for (int iSeq = 0; iSeq < sequenceFileNames.size(); iSeq++)
			{
				sceneSequenceFileName = (char *)(sequenceFileNames[iSeq].data());
				sceneFolder = std::string(sceneSequenceFileName, std::string(sceneSequenceFileName).rfind(RVLFILEPATH_SEPARATOR_) + 1);

			FileSequenceLoader sequenceLoader;
			if (!detector.bLoadMovingPartHypothesesFromFile || !detector.bLoadDDHypothesesFromFile || detector.GetVisualizeDoorHypotheses())
			{
				printf("Loading image sequence...\n");
				sequenceLoader.Init(sceneSequenceFileName);
			}
			char imgFilePath[200];
			char imgFileName[200];

			// Create models.

			if (detector.test == RVLDDD_TEST_DDD)
			{
				printf("Loading models...\n");
				FileSequenceLoader modelLoader;
				modelLoader.Init(modelSequenceFileName);
				char modelFilePath[200];
				char modelFileName[200];
				while (modelLoader.GetNext(modelFilePath, modelFileName))
					modelFileNames.push_back(modelFilePath);
				detector.LoadModels(modelFileNames);
				printf("completed.\n");
			}
			else if (detector.test == RVLDDD_TEST_CUBOIDS || detector.test == RVLDDD_TEST_DDD2 || detector.test == RVLDDD_TEST_SVD)
			{
				RVL_DELETE_ARRAY(detector.models.Element);
				detector.models.Element = new RECOG::DDD::Model[1];
				detector.models.n = 1;
				if (detector.test == RVLDDD_TEST_CUBOIDS)
				{
					float cuboidSize[] = {1.0f, 1.0f, 1.0f};
					detector.CreateCuboidModel(cuboidSize, 0.1f, detector.models.Element);
				}
				else if (detector.test == RVLDDD_TEST_DDD2)
				{
					float cuboidSize[] = {0.4f, 0.2f, 0.018f};
					// detector.CreateCuboidModel(cuboidSize, 0.02f, detector.models.Element);
					detector.CreateCuboidModel2(cuboidSize, 1.0f / 0.02f, detector.models.Element);
				}
				else if (detector.test == RVLDDD_TEST_SVD)
					detector.CreateStorageVolumeModel(detector.models.Element);
				else
					printf("ERROR: False test ID!\n");
			}

			if (detector.test == RVLDDD_TEST_DDD || detector.test == RVLDDD_TEST_CUBOIDS || detector.test == RVLDDD_TEST_DDD2 ||
				detector.test == RVLDDD_TEST_SVD || detector.test == RVLDDD_TEST_DETECT_RECTSTRUCT || detector.test == RVLDDD_TEST_RECOGNIZE_RECTSTRUCT ||
				detector.test == RVLDDD_TEST_RECOGNIZE_AO)
			{
				// Load query meshes.

				if (detector.bLoadMovingPartHypothesesFromFile && detector.bLoadDDHypothesesFromFile && !detector.GetVisualizeDoorHypotheses())
					meshSeq.n = 0;
				else
				{
					meshSeq.Element = new Mesh[sequenceLoader.nFileNames];
					pMesh = meshSeq.Element;
					sceneTransformations.Element = new RECOG::DDD::Transformation[sequenceLoader.nFileNames];
					pSceneTransformation = sceneTransformations.Element;
					while (sequenceLoader.GetNext(imgFilePath, imgFileName))
					{
						printf("Loading image %s\n", imgFileName);
						IOFileNames(sceneFolder, imgFileName, RGBImageFolder, depthImageFolder, PLYFolder, transformationsFolder, RGBFileName, depthFileName, PLYFileName, transformationsFileName);
#ifdef RVLPCL
						if (bMeshBuilder)
							LoadMesh(&meshBuilder, (char *)(PLYFileName.data()), pMesh++, false);
						else
#endif
						{
							pMesh->LoadFromPLY((char *)(PLYFileName.data()), maxMeshTriangleEdgeLen, true, &camera);
							pMesh++;
						}

						detector.LoadIRITransformationsFromYAML(transformationsFileName, pSceneTransformation->R, pSceneTransformation->t);
						pSceneTransformation++;
						if (detector.GetRGBImageVisualization())
							RGBSeq.push_back(cv::imread(RGBFileName));
					}
					meshSeq.n = pMesh - meshSeq.Element;
					sceneTransformations.n = pSceneTransformation - sceneTransformations.Element;
					printf("completed.\n");
				}

				if (detector.test == RVLDDD_TEST_CUBOIDS)
				{
					// Detect cuboids.

					for (int iScene = 0; iScene < meshSeq.n; iScene++)
						detector.DetectCuboids(meshSeq.Element + iScene);
				}
				else if (detector.test == RVLDDD_TEST_SVD)
				{
					// Detect sotrage volumes.

					for (int iScene = 0; iScene < meshSeq.n; iScene++)
						detector.DetectStorageVolumes(meshSeq.Element + iScene);
				}
				else if (detector.test == RVLDDD_TEST_DETECT_RECTSTRUCT)
				{
					// Detect rectangular structures.

					RECOG::DDD::RectStruct rectStruct;
					for (int iScene = 0; iScene < meshSeq.n; iScene++)
					{
						detector.RectangularStructures(meshSeq.Element + iScene, &rectStruct);
						delete[] rectStruct.rects.Element;
					}
				}
				else if (detector.test == RVLDDD_TEST_RECOGNIZE_RECTSTRUCT)
				{
					FileSequenceLoader modelLoader;
					modelLoader.Init(modelSequenceFileName);
					Array<RECOG::DDD::RectStruct> models;
					models.n = modelLoader.nFileNames;
					models.Element = new RECOG::DDD::RectStruct[models.n];
					char modelFilePath[200];
					char modelFileName[200];
					int iModel = 0;
					while (modelLoader.GetNext(modelFilePath, modelFileName))
					{
						std::string modelInFileName = modelFilePath;
						std::string modelOutFileName = modelInFileName.substr(0, modelInFileName.rfind('.') + 1) + "dat";
						detector.LoadRectangularStructure(modelOutFileName, models.Element + iModel);
						iModel++;
					}

					/// Recognize the model rectangular structure in the scenes.

					RECOG::DDD::RectStruct rectStruct;
					for (int iScene = 0; iScene < meshSeq.n; iScene++)
					{
						// Detect rectangular structure.

						if (detector.RectangularStructures(meshSeq.Element + iScene, &rectStruct))
						{
							// Compare the detected rectangular structure to the model.

							Pose3D poseMQ, poseMC;
							detector.MatchRectangularStructures(models.Element, &rectStruct, poseMQ, poseMC);
						}

						delete[] rectStruct.rects.Element;
					}

					///

					for (int iModel = 0; iModel < models.n; iModel++)
						delete[] models.Element[iModel].rects.Element;
					delete[] models.Element;
				}
				else if (detector.test == RVLDDD_TEST_RECOGNIZE_AO)
				{
#ifdef NEVER
					FileSequenceLoader modelLoader;
					modelLoader.Init(modelSequenceFileName);
					char modelFilePath[200];
					char modelFileName[200];
					modelLoader.GetNext(modelFilePath, modelFileName);
					RECOG::DDD::ArticulatedObject AObj;
					detector.LoadArticulatedObject(modelFilePath, AObj);
#endif
					RECOG::DDD::ArticulatedObject AObj;
					RECOG::DDD::ArticulatedObject *pAObj;

					FileSequenceLoader modelLoader;
					modelLoader.Init(modelSequenceFileName);
					char modelFilePath[200];
					char modelFileName[200];
					int nModels = modelLoader.nFileNames;
					Array<RECOG::DDD::ArticulatedObject> AObjs;
					AObjs.Element = new RECOG::DDD::ArticulatedObject[nModels];
					AObjs.n = nModels;
					pAObj = AObjs.Element;
					Array<RECOG::DDD::Transformation> modelTransformations;
					modelTransformations.Element = new RECOG::DDD::Transformation[nModels];
					modelTransformations.n = nModels;
					RECOG::DDD::Transformation *pModelTransformation = modelTransformations.Element;
					while (modelLoader.GetNext(modelFilePath, modelFileName))
					{
						printf("Loading model %s\n", modelFileName);
						detector.LoadArticulatedObject(modelFilePath, AOFileName, *pAObj);
						detector.LoadGTTransformations(modelFilePath, *pModelTransformation);
						pAObj++;
						pModelTransformation++;
					}
					printf("completed.\n");

					Pose3D poseOC;
					char sceneFileName[200];
					char sceneFilePath[200];
					char AOModelFileName[200];
					int iAOModel = 2;
					printf("\n************************\n\n");
					for (int iScene = 0; iScene < meshSeq.n; iScene++)
					{
						sequenceLoader.GetFileName(iScene, sceneFileName);
						printf("Processing image %s:\n", sceneFileName);

						sequenceLoader.GetFilePath(iScene, sceneFilePath);
						Array<int> iAOModels;
						iAOModels.Element = new int[nModels];
						detector.SelectAOModel(sceneTransformations.Element + iScene, modelTransformations, AObjs, AObj, iAOModels);
						// iAOModels.n = 1;
						// iAOModels.Element[0] = 0;
						// AObj = AObjs.Element[iAOModels.Element[0]];
						Pose3D poseOC_MRL;
						RECOG::DDD::Transformation *pPoseC0;
						RECOG::DDD::Transformation *pPoseO0;
						if (iAOModels.n == 0)
						{
							printf("AO model not found!\n");
							break;
						}
						else
						{
							modelLoader.GetFileName(iAOModels.Element[0], AOModelFileName);
							printf("AO model %s selected!\n", AOModelFileName);
							pPoseC0 = sceneTransformations.Element + iScene;
							pPoseO0 = modelTransformations.Element + iAOModels.Element[0];
							float V3Tmp[3];
							RVLCOMPTRANSF3DWITHINV(pPoseC0->R, pPoseC0->t, pPoseO0->R, pPoseO0->t, poseOC_MRL.R, poseOC_MRL.t, V3Tmp);
						}
						cv::Mat *pRGBImg = RGBSeq.data() + iScene;
						Mesh *pMesh = meshSeq.Element + iScene;
						// RECOG::DDD::AOHypothesisState* states;
						// detector.AOZeroState(AObj, states);
						// if (detector.GetRGBImageVisualization())
						//{
						//	if (pRGBImg)
						//	{
						//		cv::Mat display;
						//		pRGBImg->copyTo(display);
						//		detector.GetVisualizer()->SetMesh(pMesh);
						//		detector.VisualizeArticulatedObject(AObj, poseOC_MRL, false, &display);
						//		cv::imshow("Articulated object", display);
						//		cv::waitKey();
						//		detector.RunVisualizer();
						//		detector.ClearVisualization();
						//	}
						// }
						float verticalMAxis[3];
						double *Z0O = pPoseO0->R + 6;
						RVLCOPY3VECTOR(Z0O, verticalMAxis);
						if (detector.RecognizeArticulatedObject(pMesh, AObj, poseOC, &poseOC_MRL, verticalMAxis, (detector.GetRGBImageVisualization() ? pRGBImg : NULL)))
							// if (detector.RecognizeArticulatedObject(pMesh, AObj, poseOC, NULL, NULL, (detector.GetRGBImageVisualization() ? pRGBImg : NULL)))
							detector.RecognizeArticulatedObjectState(pMesh, AObj, poseOC, (detector.GetRGBImageVisualization() ? pRGBImg : NULL), true);
						printf("\n************************\n\n");
					}
					// delete[] AObj.MRS.rects.Element;
					// delete[] AObj.movingParts.Element;
					for (int iAObj = 0; iAObj < AObjs.n; iAObj++)
					{
						delete[] AObjs.Element[iAObj].MRS.rects.Element;
						delete[] AObjs.Element[iAObj].movingParts.Element;
					}
					delete[] AObjs.Element;
					delete[] modelTransformations.Element;
					RVL_DELETE_ARRAY(sceneTransformations.Element);
				}
				else
				{
					// Detect drawers and doors.

					std::string hypFileName = std::string(sceneSequenceFileName);
					hypFileName = hypFileName.substr(0, hypFileName.rfind(RVLFILEPATH_SEPARATOR_) + 1) + "hyps.txt";
					RECOG::DDD::HypothesisDoorDrawer DDObject;
					detector.Detect(meshSeq, &DDObject, (char *)hypFileName.data(), (detector.GetRGBImageVisualization() ? &RGBSeq : NULL));
					// detector.Detect2(meshSeq);

					// // ************ IoU Hypothesis evaluation ************
					// // std::string csvFileName = hypFileName.substr(0, hypFileName.rfind(pathSeparator) + 1) + "bbox_gt/gt.csv";
					// // cout << csvFileName << endl;
					// // std::vector<std::vector<std::string>> csvContent;
					// // std::vector<std::vector<PSD::Point2D>> allGTPoints;

					// // cv::Mat pRGBDisplay = cv::imread(hypFileName.substr(0, hypFileName.rfind(pathSeparator) + 1) + "rgb/0000.png");

					// if (detector.ParseCSV(csvFileName, true, csvContent))
					// {
					// 	// detector.GetGTPointsFromCSV(csvContent, allGTPoints);
					// 	float *iouResults = new float[allGTPoints.size()];
					// 	detector.cvIoUHypothesisEvaluation(DDObject, allGTPoints, iouResults, &pRGBDisplay);

					// 	for (int iRes = 0; iRes < allGTPoints.size(); iRes++)
					// 		cout << "IoU: " << iouResults[iRes] << endl;
					// 	delete[] iouResults;
					// 	}
					// 	cv::imwrite(hypFileName.substr(0, hypFileName.rfind(RVLFILEPATH_SEPARATOR_) + 1) + "bbox_gt/gt.png", pRGBDisplay);
				}
			}
			else if (detector.test == RVLDDD_TEST_3DTO2DFIT)
			{
				RVL::Pose3D poseC_C0[2];
					if (flags & RVLRECOGNITION_DEMO_FLAG_STEREO_CALIBRATE)
						detector.StereoCalibrationFromImages(stereoCalibrationPath, camsMatrix[0], camsDist[0], camsMatrix[1], camsDist[1], &poseC_C0[0]);
					else
						detector.loadBaslerExtrinsicParams(stereoCalibrationPath, &poseC_C0[0]);

				// Create model.

				Mesh modelMesh;
				if (detector.edgeModel == RVLDDD_EDGE_MODEL_BOX)
				{
					float boxSize[] = {0.287f, 0.210f, 0.090f};
					detector.CreateBox(&modelMesh, boxSize);
				}
				else if (detector.edgeModel == RVLDDD_EDGE_MODEL_CYLINDER)
				{
					// float rCylinder = 0.015f;
					// float hCylinder = 0.800f;
					// detector.CreateCylinder(&modelMesh, rCylinder, hCylinder, 20);
					float rCylinder = 0.015f;
					float hCylinder = 0.100f;
					float rCylinder2 = 0.010f;
					float hCylinder2 = 0.700f;
					detector.CreateLHTCPModel(&modelMesh, rCylinder, hCylinder, rCylinder2, hCylinder2, 20);
				}
				detector.CreateMeshFromPolyData(&modelMesh);

				// Initial pose.
				Pose3D poseMCInit0, poseMCInit, poseMC;
				Vector3<float> *poseCorrection;
				Array<Pose3D> initPoses;
				Pose3D initPosesMem[4];
				initPoses.Element = initPosesMem;
				initPoses.n = 4;
				if (detector.edgeModel == RVLDDD_EDGE_MODEL_BOX)
				{
					float pan = 45.0f * DEG2RAD;
					float tilt = -45.0f * DEG2RAD;
					float csAlpha = cos(pan);
					float snAlpha = sin(pan);
					float csBeta = cos(tilt);
					float snBeta = sin(tilt);
					float RSC0[9] = {0.0f, -1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 1.0f, 0.0f, 0.0f};
					float RMS[9];
					RVLROTZ(csAlpha, snAlpha, RMS);
					float tMS[3];
					RVLSET3VECTOR(tMS, 0.5f, 0.0f, -0.6f);
					float RCC0[9];
					RVLROTX(csBeta, snBeta, RCC0);
					float RSC[9];
					RVLMXMUL3X3T1(RCC0, RSC0, RSC);
					RVLMXMUL3X3(RSC, RMS, poseMCInit0.R);
					RVLMULMX3X3VECT(RSC, tMS, poseMCInit0.t);
					float poseCorrectionBox[10][3] = {{0.0, 0.0, 0.0},
													  {-0.1, 0.0, 0.0},
													  {-0.1, 0.0, 0.0},
													  {0.20, 0.0, 0.0},
													  {0.0, 0.0, 0.0},
													  {-0.07, 0.0, 0.0},
													  {0.05, 0.0, 0.05},
													  {0.04, 0.0, -0.03},
													  {-0.04, -0.10, -0.04},
													  {0.0, 0.0, 0.0}};
					poseCorrection = new Vector3<float>[10];
					for (int i = 0; i < 10; i++)
					{
						RVLCOPY3VECTOR(poseCorrectionBox[i], poseCorrection[i].Element);
					}
				}
				else if (detector.edgeModel == RVLDDD_EDGE_MODEL_CYLINDER)
				{
					// RVLROTX(0.0f, 1.0f, poseMCInit0.R);
					// RVLSET3VECTOR(poseMCInit0.t, 0.025f, 0.30f, 0.40f);
						float angle = 20.0f;
					for (int i = 0; i < initPoses.n / 2; i++)
					{
						// float cs = cos(-70.0 * DEG2RAD);
						// float sn = sin(-70.0 * DEG2RAD);
							float th = -0.5f * PI + (float)(2 * i - 1) * angle * DEG2RAD;
						float cs = cos(th);
						float sn = sin(th);
						RVLROTX(cs, sn, poseMCInit0.R);
							RVLSET3VECTOR(poseMCInit0.t, 0.0f, 0.0f, 0.28f);
						initPoses.Element[i] = poseMCInit0;
					}
					float th, cs, sn, csx, snx;
					float rotX[9], rotZ[9];
					for (int i = 0; i < initPoses.n / 2; i++) // rotz
					{
						csx = cos(-0.5f * PI);
						snx = sin(-0.5f * PI);
						RVLROTX(csx, snx, rotX);

							th = (float)(2 * i - 1) * angle * DEG2RAD;
						cs = cos(th);
						sn = sin(th);
						RVLROTZ(cs, sn, rotZ);

						RVLMXMUL3X3(rotZ, rotX, poseMCInit0.R);

							RVLSET3VECTOR(poseMCInit0.t, 0.0f, 0.0f, 0.28f);
						initPoses.Element[i + 2] = poseMCInit0;
					}
					float poseCorrectionBox[6][3] = {{-0.010, 0.0, 0.0},
													 {-0.01, 0.0, 0.0},
													 {-0.01, 0.0, 0.0},
													 {-0.02, 0.0, 0.0},
													 {-0.02, 0.0, 0.0},
													 {0.0, 0.0, 0.0}};
					poseCorrection = new Vector3<float>[6];
					for (int i = 0; i < 6; i++)
					{
						RVLCOPY3VECTOR(poseCorrectionBox[i], poseCorrection[i].Element);
					}
				}

				// Apply to images.

				int iImage = 0;
				cv::Mat RGBs[2];
				cv::Mat RGBsUndistorted[2];
				cv::Mat *solutions;
				solutions = new cv::Mat[2];
					int counter = 0;
					RVL::Pose3D referentPoseMC, currentPoseMCX, currentPoseMCY, resultPoseMC, tempPoseMC;
					RVL::Pose3D resultPoseWC, resultPoseCW;
					float V[3];
					float theta;

					if (1)
					{
				while (sequenceLoader.GetNext(imgFilePath, imgFileName))
				{
					printf("Image %s\n", imgFileName);
					if (detector.fit3DTo2DStereo)
					{
						std::string imaFilePath_ = imgFilePath;
						// RGBs[0] = cv::imread((imaFilePath_ + "-1.png").data());
						// RGBs[1] = cv::imread((imaFilePath_ + "-2.png").data());
						for (int i = 0; i < 2; i++)
						{
							RGBs[i] = cv::imread((imaFilePath_ + "-" + std::to_string(i) + ".png").data());
							cv::undistort(RGBs[i], RGBsUndistorted[i], camsMatrix[i], camsDist[i]);
						}
						// VisualizeEpipolarGeometry(RGBsUndistorted[0], RGBsUndistorted[1], camsMatrix[0], camsMatrix[1], poseC_C0[1]);
								detector.Fit3DTo2DStereo(&modelMesh, &RGBsUndistorted[0], 2, initPoses, poseC_C0, poseMC, solutions, counter);
								// counter++;
								counter++;
					}
					else
					{
						cv::Mat RGB = cv::imread(imgFilePath);
						// cv::imshow(imgFileName, RGB);
						// cv::waitKey();
						poseMCInit = poseMCInit0;
						// RVLSUM3VECTORS(poseMCInit.t, poseCorrection[iImage].Element, poseMCInit.t);
						// detector.Fit3DTo2D(&modelMesh, RGB, poseMCInit, poseMC);
						detector.Fit3DTo2D(&modelMesh, RGB, initPoses, poseMC);
						if (detector.edgeModel == RVLDDD_EDGE_MODEL_BOX)
							poseMCInit0 = poseMC;
					}
					iImage++;
				}
					}
					// else
					// {
					// 	// // LHTCP measurements
					// 	std::string imagesPath = "/home/RVLuser/rvl-linux/data/DANIELI_LHTCP/experiments/Exp-LHTCP_measurements-230201/images";
					// 	for (int i = 0; i < 2; i++)
					// 	{
					// 		RGBs[i] = cv::imread((imagesPath + "/12" + "-" + std::to_string(i) + ".png").data());
					// 		cv::undistort(RGBs[i], RGBsUndistorted[i], camsMatrix[i], camsDist[i]);
					// 	}
					// 	referentPoseMC = detector.Fit3DTo2DStereo2(&modelMesh, &RGBsUndistorted[0], 2, initPoses, poseC_C0, poseMC, solutions, counter);

					// 	cout << endl
					// 		 << "TMC0: " << endl;

					// 	for (const auto &e : referentPoseMC.R)
					// 	{
					// 		cout << e << ", ";
					// 	}
					// 	cout << endl;
					// 	for (const auto &e : referentPoseMC.t)
					// 	{
					// 		cout << e << ", ";
					// 	}
					// 	cout << endl;

					// 	for (int i = 0; i < 2; i++)
					// 	{
					// 		RGBs[i] = cv::imread((imagesPath + "/15" + "-" + std::to_string(i) + ".png").data());
					// 		cv::undistort(RGBs[i], RGBsUndistorted[i], camsMatrix[i], camsDist[i]);
					// 	}
					// 	currentPoseMCX = detector.Fit3DTo2DStereo2(&modelMesh, &RGBsUndistorted[0], 2, initPoses, poseC_C0, poseMC, solutions, counter);

					// 	float dist = sqrt(pow(referentPoseMC.t[0] - currentPoseMCX.t[0], 2) + pow(referentPoseMC.t[1] - currentPoseMCX.t[1], 2) + pow(referentPoseMC.t[2] - currentPoseMCX.t[2], 2));
					// 	cout << "Distance: " << 0.15 - dist << endl;

					// 	return 0;

					// 	cout << endl
					// 		 << "TMC0X: " << endl;
					// 	for (const auto &e : currentPoseMCX.R)
					// 	{
					// 		cout << e << ", ";
					// 	}
					// 	cout << endl;
					// 	for (const auto &e : currentPoseMCX.t)
					// 	{
					// 		cout << e << ", ";
					// 	}
					// 	cout << endl;

					// 	// for (int i = 0; i < 2; i++)
					// 	// {
					// 	// 	RGBs[i] = cv::imread((imagesPath + "/4" + "-" + std::to_string(i) + ".png").data());
					// 	// 	cv::undistort(RGBs[i], RGBsUndistorted[i], camsMatrix[i], camsDist[i]);
					// 	// }
					// 	// currentPoseMCY = detector.Fit3DTo2DStereo2(&modelMesh, &RGBsUndistorted[0], 2, initPoses, poseC_C0, poseMC, solutions, counter);

					// 	// cout << endl << "TMC0Y: " << endl;
					// 	// for (const auto &e : currentPoseMCY.R)
					// 	// {
					// 	// 	cout << e << ", ";
					// 	// }
					// 	// cout << endl;
					// 	// for (const auto &e : currentPoseMCY.t)
					// 	// {
					// 	// 	cout << e << ", ";
					// 	// }
					// 	// cout << endl;

					// 	// float tempX[3], tempY[3];
					// 	// RVLDIF3VECTORS(currentPoseMCX.t, referentPoseMC.t, tempPoseMC.t);
					// 	// RVLCOPYTOCOL3(tempPoseMC.t, 0, resultPoseWC.R);
					// 	// RVLDIF3VECTORS(currentPoseMCY.t, referentPoseMC.t, tempPoseMC.t);
					// 	// RVLCOPYTOCOL3(tempPoseMC.t, 1, resultPoseWC.R);

					// 	// RVLCOPYCOLMX3X3(resultPoseWC.R, 0, tempX);
					// 	// RVLCOPYCOLMX3X3(resultPoseWC.R, 1, tempY);
					// 	// RVLCROSSPRODUCT3(tempX, tempY, tempPoseMC.t);
					// 	// RVLCOPYTOCOL3(tempPoseMC.t, 2, resultPoseWC.R);

					// 	// // imagesPath = "/home/RVLuser/rvl-linux/data/DANIELI_LHTCP/experiments/Exp-LHTCP_measurements-230206/images";
					// 	// RVL::Pose3D rotatingPoseMC, rotatingPoseM_C;
					// 	// for (int i = 0; i < 2; i++)
					// 	// {
					// 	// 	RGBs[i] = cv::imread((imagesPath + "/8" + "-" + std::to_string(i) + ".png").data());
					// 	// 	cv::undistort(RGBs[i], RGBsUndistorted[i], camsMatrix[i], camsDist[i]);
					// 	// }
					// 	// rotatingPoseMC = detector.Fit3DTo2DStereo2(&modelMesh, &RGBsUndistorted[0], 2, initPoses, poseC_C0, poseMC, solutions, counter);
					// 	// cout << endl<< "rotTMC0: " << endl;
					// 	// for (const auto &e : rotatingPoseMC.R)
					// 	// {
					// 	// 	cout << e << ", ";
					// 	// }
					// 	// cout << endl;
					// 	// for (const auto &e : rotatingPoseMC.t)
					// 	// {
					// 	// 	cout << e << ", ";
					// 	// }
					// 	// cout << endl;

					// 	// for (int i = 0; i < 2; i++)
					// 	// {
					// 	// 	RGBs[i] = cv::imread((imagesPath + "/11" + "-" + std::to_string(i) + ".png").data());
					// 	// 	cv::undistort(RGBs[i], RGBsUndistorted[i], camsMatrix[i], camsDist[i]);
					// 	// }
					// 	// rotatingPoseM_C = detector.Fit3DTo2DStereo2(&modelMesh, &RGBsUndistorted[0], 2, initPoses, poseC_C0, poseMC, solutions, counter);

					// 	// cout << endl << "rotTM_C: " << endl;
					// 	// for (const auto &e : rotatingPoseM_C.R)
					// 	// {
					// 	// 	cout << e << ", ";
					// 	// }
					// 	// cout << endl;
					// 	// for (const auto &e : rotatingPoseM_C.t)
					// 	// {
					// 	// 	cout << e << ", ";
					// 	// }
					// 	// cout << endl;

					// 	// float zMC[3], zM_C[3], zMW[3], zM_W[3];
					// 	// RVLCOPYCOLMX3X3(rotatingPoseMC.R, 2, zMC);
					// 	// RVLCOPYCOLMX3X3(rotatingPoseM_C.R, 2, zM_C);
					// 	// RVLMULMX3X3TVECT(resultPoseWC.R, zMC, zMW);
					// 	// RVLMULMX3X3TVECT(resultPoseWC.R, zM_C, zM_W);

					// 	// float tempLen;
					// 	// zMW[2] = 0.0;
					// 	// zM_W[2] = 0.0;

					// 	// cout << acos((zMW[0]*zM_W[0]+zMW[1]*zM_W[1])/(sqrt(RVLDOTPRODUCT3(zMW, zMW))*sqrt(RVLDOTPRODUCT3(zM_W, zM_W)))) * RAD2DEG << endl;
					// }

				delete[] poseCorrection;
				delete[] solutions;
			}
			else
				printf("Test not defined!\n");

				RVL_DELETE_ARRAY(meshSeq.Element);
			}
		}

		// Free memory.
		RVL_DELETE_ARRAY(camsMatrix);
		RVL_DELETE_ARRAY(camsDist);
	}
    else if (method == RVLRECOGNITION_METHOD_BM)
    {
        printf("Creating Branch Matcher...\n");
        BranchMatcher detector;
        detector.pMem0 = &mem0;
        detector.Create(cfgFileName);
        detector.camera = camera;
        detector.InitVisualizer(&visualizer);
        FileSequenceLoader sequenceLoader;
        printf("Loading image sequence %s...\n", sceneSequenceFileName);
        sequenceLoader.Init(sceneSequenceFileName);
        char imgFilePath[200];
        char imgFileName[200];
        while (sequenceLoader.GetNext(imgFilePath, imgFileName))
        {
            printf("Loading image %s\n", imgFileName);
            IOFileNames(sceneFolder, imgFileName, RGBImageFolder, depthImageFolder, PLYFolder, transformationsFolder, RGBFileName, depthFileName, PLYFileName, transformationsFileName);
            Array2D<short int> depthImage;
            cv::Mat depthImagePNG = cv::imread(depthFileName, CV_LOAD_IMAGE_ANYDEPTH);
            depthImage.w = depthImagePNG.cols;
            depthImage.h = depthImagePNG.rows;
            depthImage.Element = (short int *)(depthImagePNG.data);
            // IplImage* depthImageDisplay = cvCreateImage(cvSize(depthImage.w, depthImage.h), IPL_DEPTH_8U, 3);
            // DisplayDisparityMap(depthImage, (unsigned char*)(depthImageDisplay->imageData), true, RVLRGB_DEPTH_FORMAT_1MM);
            // cv::namedWindow(imgFileName);
            // cvShowImage(imgFileName, depthImageDisplay);
            // cv::waitKey();
            detector.Detect(depthImage);
        }
    }

	// free memory

	delete[] cfgFileName;

	RVL_DELETE_ARRAY(sceneMeshFileName);
	// RVL_DELETE_ARRAY(sceneSequenceFileName);
	RVL_DELETE_ARRAY(modelSequenceFileName);
	RVL_DELETE_ARRAY(cameraParamsFileName);
	RVL_DELETE_ARRAY(stereoCalibrationPath);
	RVL_DELETE_ARRAY(modelsInDB); // VIDOVIC
	RVL_DELETE_ARRAY(GTFolder);	  // VIDOVIC
	RVL_DELETE_ARRAY(instanceFileName);
	RVL_DELETE_ARRAY(primitiveFileName);
	RVL_DELETE_ARRAY(rotInvarianceInfoFileName);

	// END VIDOVIC

#ifdef RVLOPENNI
	if (cameraType == RVLCAMERA_ASTRA)
	{
		astra.TerminateAstra();
	}
#endif

	return 0;
}

// Only for debugging purpose!!!
//
// void RVLMinTest()
//{
//	Array<SortIndex<float>> dataArray;
//
//	dataArray.n = 10000;
//	dataArray.Element = new SortIndex<float>[dataArray.n];
//
//	int nTopData = 10;
//
//	Array<SortIndex<float>> topDataArray;
//
//	topDataArray.n = nTopData;
//	topDataArray.Element = new SortIndex<float>[topDataArray.n];
//
//	FILE *fpSrc, *fpTgt;
//
//	for (int i = 0; i < 20; i++)
//	{
//		fpSrc = fopen("a.txt", "w");
//
//		for (int j = 0; j < dataArray.n; j++)
//		{
//			dataArray.Element[j].idx = j;
//			dataArray.Element[j].cost = (float)rand() / (float)RAND_MAX;
//
//			fprintf(fpSrc, "%f\n", dataArray.Element[j].cost);
//		}
//
//		fclose(fpSrc);
//
//		Min<SortIndex<float>, float>(dataArray, nTopData, topDataArray);
//
//		fpTgt = fopen("b.txt", "w");
//
//		for (int j = 0; j < topDataArray.n; j++)
//			fprintf(fpSrc, "%d\t%f\n", topDataArray.Element[j].idx, topDataArray.Element[j].cost);
//
//		fclose(fpTgt);
//
//		int debug = 0;
//	}
//
//	delete[] dataArray.Element;
//	delete[] topDataArray.Element;
//}

#ifdef RVLPCL
void GenerateSegmentNeighbourhood(PSGM *psgm, double radius)
{
	psgm->segmentN_PD.clear();

	// delete old KDTree
	std::map<int, void *>::iterator iKdTree;

	for (iKdTree = psgm->segmentN_KdTree.begin(); iKdTree != psgm->segmentN_KdTree.end(); iKdTree++)
		delete (pcl::search::KdTree<pcl::PointXYZINormal> *)iKdTree->second;

	// clear old kdTree pointers array
	psgm->segmentN_KdTree.clear();

	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_destination(new pcl::PointCloud<pcl::PointXYZINormal>);
	// creating PCL point cloud
	cloud_destination->width = psgm->pMesh->NodeArray.n;
	cloud_destination->height = 1;
	cloud_destination->is_dense = false;
	cloud_destination->points.resize(cloud_destination->width * cloud_destination->height);

	int idx = 0;
	for (int i = 0; i < psgm->pMesh->NodeArray.n; i++)
	{
		if (psgm->clusterMap[psgm->pSurfels->surfelMap[i]] == -1)
			continue;

		cloud_destination->points[idx].x = psgm->pMesh->NodeArray.Element[i].P[0];
		cloud_destination->points[idx].y = psgm->pMesh->NodeArray.Element[i].P[1];
		cloud_destination->points[idx].z = psgm->pMesh->NodeArray.Element[i].P[2];

		cloud_destination->points[idx].normal_x = psgm->pMesh->NodeArray.Element[i].N[0];
		cloud_destination->points[idx].normal_y = psgm->pMesh->NodeArray.Element[i].N[1];
		cloud_destination->points[idx].normal_z = psgm->pMesh->NodeArray.Element[i].N[2];

		idx++;
	}

	pcl::search::KdTree<pcl::PointXYZINormal>::Ptr kdtree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZINormal>>((new pcl::search::KdTree<pcl::PointXYZINormal>));
	kdtree->setInputCloud(cloud_destination);

	RECOG::PSGM_::Cluster *pCluster;
	Surfel *pSurfel;
	RVL::QLIST::Index2 *pt;

	// Finding centroids
	float *centroids = new float[3 * psgm->clusters.n];
	memset(centroids, 0, 3 * psgm->clusters.n * sizeof(float));
	int noPts;
	for (int iCluster = 0; iCluster < psgm->clusters.n; iCluster++)
	{
		pCluster = psgm->clusters.Element[iCluster];
		noPts = 0;
		for (int i = 0; i < pCluster->iSurfelArray.n; i++)
		{
			pSurfel = &psgm->pSurfels->NodeArray.Element[pCluster->iSurfelArray.Element[i]];
			pt = pSurfel->PtList.pFirst;
			while (pt)
			{
				centroids[3 * iCluster] += psgm->pMesh->NodeArray.Element[pt->Idx].P[0];
				centroids[3 * iCluster + 1] += psgm->pMesh->NodeArray.Element[pt->Idx].P[1];
				centroids[3 * iCluster + 2] += psgm->pMesh->NodeArray.Element[pt->Idx].P[2];
				noPts++;
				pt = pt->pNext;
			}
		}
		centroids[3 * iCluster] /= noPts;
		centroids[3 * iCluster + 1] /= noPts;
		centroids[3 * iCluster + 2] /= noPts;
	}

	std::vector<int> pointIdxRadiusSearch;		   // to store index of surrounding points
	std::vector<float> pointRadiusSquaredDistance; // to store distance to surrounding points
	pcl::PointXYZINormal searchPoint;
	vtkSmartPointer<vtkPoints> points;
	vtkSmartPointer<vtkFloatArray> normals;
	vtkSmartPointer<vtkCellArray> verts;
	vtkSmartPointer<vtkPolyData> PD;
	int ptIdx = 0;
	for (int iCluster = 0; iCluster < psgm->clusters.n; iCluster++)
	{
		points = vtkSmartPointer<vtkPoints>::New();
		normals = vtkSmartPointer<vtkFloatArray>::New();
		normals->SetNumberOfComponents(3);
		verts = vtkSmartPointer<vtkCellArray>::New();
		searchPoint.x = centroids[3 * iCluster];
		searchPoint.y = centroids[3 * iCluster + 1];
		searchPoint.z = centroids[3 * iCluster + 2];

		pointIdxRadiusSearch.clear();
		pointRadiusSquaredDistance.clear();
		kdtree->radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
		ptIdx = 0;
		for (int i = 0; i < pointIdxRadiusSearch.size(); i++)
		{
			points->InsertNextPoint(cloud_destination->points[pointIdxRadiusSearch.at(i)].x, cloud_destination->points[pointIdxRadiusSearch.at(i)].y, cloud_destination->points[pointIdxRadiusSearch.at(i)].z);
			normals->InsertNextTuple(cloud_destination->points[pointIdxRadiusSearch.at(i)].normal);
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

		psgm->segmentN_PD.insert(std::make_pair(iCluster, cleanFilter->GetOutput()));

#ifdef RVLPSGM_ICP_IN_SCENE_CS
		// Generate KdTree for segment neighbourhood points
		vtkSmartPointer<vtkPoints> pdPoints = cleanFilter->GetOutput()->GetPoints();
		vtkSmartPointer<vtkFloatArray> normals = vtkFloatArray::SafeDownCast(cleanFilter->GetOutput()->GetPointData()->GetNormals());
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
		cloud->width = pdPoints->GetNumberOfPoints();
		cloud->height = 1;
		cloud->is_dense = false;
		cloud->points.resize(cloud->width * cloud->height);
		double *point;
		float normal[3];
		bool hasNormals = false;
		if (normals.GetPointer()) // check if exists
			hasNormals = true;
		for (int i = 0; i < pdPoints->GetNumberOfPoints(); i++)
		{
			point = pdPoints->GetPoint(i);
			cloud->points[i].x = point[0];
			cloud->points[i].y = point[1];
			cloud->points[i].z = point[2];
			if (hasNormals) // check if exists
			{
				normals->GetTupleValue(i, normal);
				cloud->points[i].normal_x = normal[0];
				cloud->points[i].normal_y = normal[1];
				cloud->points[i].normal_z = normal[2];
			}
		}

		pcl::search::KdTree<pcl::PointXYZINormal> *kdtree_n = new pcl::search::KdTree<pcl::PointXYZINormal>();
		kdtree_n->setInputCloud(cloud);

		psgm->segmentN_KdTree.insert(std::make_pair(iCluster, kdtree_n));
		// END - Generate KdTree for segment neighbourhood points
#endif
	}
	delete[] centroids;
}

void CreateSceneKdTree(PSGM *psgm)
{
	// pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_destination(new pcl::PointCloud<pcl::PointXYZINormal>);
	////creating PCL point cloud
	// cloud_destination->width = psgm->pMesh->NodeArray.n;
	// cloud_destination->height = 1;
	// cloud_destination->is_dense = false;
	// cloud_destination->points.resize(cloud_destination->width * cloud_destination->height);

	// int idx = 0;
	// for (int i = 0; i <psgm->pMesh->NodeArray.n; i++)
	//{
	//	if (psgm->clusterMap[psgm->pSurfels->surfelMap[i]] == -1)
	//		continue;

	//	cloud_destination->points[idx].x = psgm->pMesh->NodeArray.Element[i].P[0];
	//	cloud_destination->points[idx].y = psgm->pMesh->NodeArray.Element[i].P[1];
	//	cloud_destination->points[idx].z = psgm->pMesh->NodeArray.Element[i].P[2];

	//	cloud_destination->points[idx].normal_x = psgm->pMesh->NodeArray.Element[i].N[0];
	//	cloud_destination->points[idx].normal_y = psgm->pMesh->NodeArray.Element[i].N[1];
	//	cloud_destination->points[idx].normal_z = psgm->pMesh->NodeArray.Element[i].N[2];

	//	idx++;
	//}

	// pcl::search::KdTree<pcl::PointXYZINormal>* kdtree = new pcl::search::KdTree<pcl::PointXYZINormal>();
	// kdtree->setInputCloud(cloud_destination);

	vtkSmartPointer<vtkPoints> pdPoints = psgm->pMesh->pPolygonData->GetPoints();
	vtkSmartPointer<vtkFloatArray> normals = vtkFloatArray::SafeDownCast(psgm->pMesh->pPolygonData->GetPointData()->GetNormals());
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
	cloud->width = pdPoints->GetNumberOfPoints();
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->points.resize(cloud->width * cloud->height);
	double *point;
	float normal[3];
	bool hasNormals = false;
	if (normals.GetPointer()) // check if exists
		hasNormals = true;
	for (int i = 0; i < pdPoints->GetNumberOfPoints(); i++)
	{
		// if (psgm->clusterMap[psgm->pSurfels->surfelMap[i]] == -1)
		//{
		//	pdPoints->SetPoint(i, 0, 0, 0);
		//	//continue;
		// }

		point = pdPoints->GetPoint(i);
		cloud->points[i].x = point[0];
		cloud->points[i].y = point[1];
		cloud->points[i].z = point[2];
		if (hasNormals) // check if exists
		{
			normals->GetTypedTuple(i, normal);
			cloud->points[i].normal_x = normal[0];
			cloud->points[i].normal_y = normal[1];
			cloud->points[i].normal_z = normal[2];
		}
	}

	pcl::search::KdTree<pcl::PointXYZINormal> *kdtree = new pcl::search::KdTree<pcl::PointXYZINormal>();
	kdtree->setInputCloud(cloud);

	psgm->pKdTree = kdtree;
}
#endif

#ifdef RVLPSGM_CUDA_ICP
// sceneDepth is depth image of models (ZBuffer)
void RunCUDAICPv1(unsigned short *modelDepth, float *T)
{
	// ONLY FOR DEBUG
	// cv::Mat depthImage(240, 320, CV_16UC1);
	// memcpy(depthImage.data, modelDepth, 320 * 240 * sizeof(unsigned short));
	// cv::imwrite("C:\\RVL\\test2.png", depthImage);

	pCUDAICPObjv1->SetIcpScene(modelDepth);
	Eigen::Matrix4f pose;
	pose = Eigen::Matrix4f::Identity();
	pCUDAICPObjv1->CalcIncrementalTransformation(pose);
	memcpy(T, pose.data(), 16 * sizeof(float));
}

// sceneDepth is depth image of models (ZBuffer)
// void RunCUDAICPv2(unsigned short *modelDepth, float *T)
//{
//	pCUDAICPObjv2->SetIcpScene(modelDepth);
//	Sophus::SE3d pose;
//	pCUDAICPObjv2->CalcIncrementalTransformation(pose);
//	memcpy(T, pose.matrix().data(), 16 * sizeof(float));
// }
#endif
