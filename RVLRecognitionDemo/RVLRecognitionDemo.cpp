// RVLRecognitionDemo.cpp : Defines the entry point for the console application.
//
#include <Windows.h>
#include <ctime>
//#include "stdafx.h"
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
//VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
#include "RVLVTK.h"
#include "RVLCore2.h"
#include "Util.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
#include "SurfelGraph.h"
#include "ObjectGraph.h"
#include "PlanarSurfelDetector.h"
#include "RVLRecognition.h"
#include "RFRecognition.h"
#include "RVLMeshNoiser.h"
#include "PSGMCommon.h"
#include "CTISet.h"
#include "PSGM.h"
#include <pcl/common/common.h>
#include <pcl/registration/registration.h>
#include <pcl/PolygonMesh.h>
#include "PCLTools.h"
#include "RGBDCamera.h"
#include "PCLMeshBuilder.h"


// VIDOVIC
//#define RVL_COORDINATE_SYSTEM_NOISE_STABILITY_TEST
//#define RVL_COORDINATE_SYSTEM_NOISE_STABILITY_TEST_DEBUG
#define RVL_FEATURE_TEST_SCENE_SEQUENCE
//#define RVL_FEATURE_TEST_PRECISION_RECALL_GRAPH
#define RVL_LOAD_SINGLE_MODEL
//#define PSGM_MATCHES_PROBABILITY_COMPARE
#define PSGM_MATCHES_SCORE_COMPARE
//#define PSGM_LOAD_CTI_FROM_FILE
//#define PSGM_RECOGNITION_VISUALIZE_SCENE

#define RVLRECOGNITION_DEMO_FLAG_SAVE_PLY			0x00000001
//END VIDOVIC

using namespace RVL;

#define RVLRECOGNITION_METHOD_RF		0
#define RVLRECOGNITION_METHOD_PSGM		1

void CreateParamList(
	CRVLParameterList *pParamList,
	CRVLMem *pMem,
	char **pMeshFileName,
	char **pSceneSequenceFileName,	//VIDOVIC
	char **pModelSequenceFileName,	//VIDOVIC
	char **pModelsInDB,	//VIDOVIC
	char **pGTFolder,	//VIDOVIC
	char **pSegmentGTFileName,	//Vidovic
	DWORD &method,
	DWORD &flags //VIDOVIC
	)
{
	pParamList->m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	pParamList->Init();

	pParamData = pParamList->AddParam("SceneFileName", RVLPARAM_TYPE_STRING, pMeshFileName);
	pParamData = pParamList->AddParam("SceneSequenceFileName", RVLPARAM_TYPE_STRING, pSceneSequenceFileName);	//VIDOVIC
	pParamData = pParamList->AddParam("ModelSequenceFileName", RVLPARAM_TYPE_STRING, pModelSequenceFileName);	//VIDOVIC
	pParamData = pParamList->AddParam("ModelsInDataBase", RVLPARAM_TYPE_STRING, pModelsInDB);	//VIDOVIC
	pParamData = pParamList->AddParam("GTFolder", RVLPARAM_TYPE_STRING, pGTFolder);	//VIDOVIC
	pParamData = pParamList->AddParam("SegmentGTFileName", RVLPARAM_TYPE_STRING, pSegmentGTFileName);	//Vidovic
	pParamData = pParamList->AddParam("Recognition.method", RVLPARAM_TYPE_ID, &method);
	pParamList->AddID(pParamData, "PSGM", RVLRECOGNITION_METHOD_PSGM);
	pParamList->AddID(pParamData, "RF", RVLRECOGNITION_METHOD_RF); //VIDOVIC
	pParamData = pParamList->AddParam("Save PLY", RVLPARAM_TYPE_ID, &flags); //VIDOVIC
	pParamList->AddID(pParamData, "yes", RVLRECOGNITION_DEMO_FLAG_SAVE_PLY); //VIDOVIC
}

void GenerateSegmentNeighbourhood(PSGM * psgm, double radius)
{
	psgm->segmentN_PD.clear();
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_destination(new pcl::PointCloud<pcl::PointXYZINormal>);
	//creating PCL point cloud
	cloud_destination->width = psgm->pMesh->NodeArray.n;
	cloud_destination->height = 1;
	cloud_destination->is_dense = false;
	cloud_destination->points.resize(cloud_destination->width * cloud_destination->height);

	for (int i = 0; i <psgm->pMesh->NodeArray.n; i++)
	{
		cloud_destination->points[i].x = psgm->pMesh->NodeArray.Element[i].P[0];
		cloud_destination->points[i].y = psgm->pMesh->NodeArray.Element[i].P[1];
		cloud_destination->points[i].z = psgm->pMesh->NodeArray.Element[i].P[2];

		cloud_destination->points[i].normal_x = psgm->pMesh->NodeArray.Element[i].N[0];
		cloud_destination->points[i].normal_y = psgm->pMesh->NodeArray.Element[i].N[1];
		cloud_destination->points[i].normal_z = psgm->pMesh->NodeArray.Element[i].N[2];
	}

	pcl::search::KdTree<pcl::PointXYZINormal>::Ptr kdtree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZINormal>>((new pcl::search::KdTree<pcl::PointXYZINormal>));
	kdtree->setInputCloud(cloud_destination);

	RECOG::PSGM_::Cluster *pCluster;
	Surfel *pSurfel;
	RVL::QLIST::Index2 *pt;

	//Finding centroids
	float *centroids = new float[3 * psgm->clusters.n];
	memset(centroids, 0, 3 * psgm->clusters.n*sizeof(float));
	int noPts;
	for (int iCluster = 0; iCluster < psgm->clusters.n; iCluster++)
	{
		pCluster = psgm->clusters.Element[iCluster];
		noPts = 0;
		for (int i = 0; i < pCluster->iSurfelArray.n; i++)
		{
			pSurfel = &psgm->pSurfels->NodeArray.Element[pCluster->iSurfelArray.Element[i]];
			pt = pSurfel->PtList.pFirst;
			for (int k = 0; k < pSurfel->size; k++)
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

	std::vector<int> pointIdxRadiusSearch; //to store index of surrounding points
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

		//subsampling the scene:
		vtkSmartPointer<vtkCleanPolyData> cleanFilter = vtkSmartPointer<vtkCleanPolyData>::New();
		cleanFilter->SetInputData(PD);
		cleanFilter->PointMergingOn();
		cleanFilter->SetAbsoluteTolerance(0.005);
		cleanFilter->ToleranceIsAbsoluteOn();
		cleanFilter->Update();

		psgm->segmentN_PD.insert(std::make_pair(iCluster, cleanFilter->GetOutput()));
	}
	delete[] centroids;
	
}

int main(int argc, char ** argv)
{
	// Create memory storage.

	CRVLMem mem0;	// permanent memory

	mem0.Create(1000000000);
	//mem0.Create(100000000000); //VIDOVIC

	CRVLMem mem;	// cycle memory

	mem.Create(1000000000);
	//mem.Create(100000000000); //VIDOVIC

	// Read parameters from a configuration file.

	char *sceneMeshFileName = NULL;
	char *sceneSequenceFileName = NULL; //VIDOVIC
	char *modelSequenceFileName = NULL; //VIDOVIC
	char *modelsInDB = NULL; //VIDOVIC
	char *GTFolder = NULL; //VIDOVIC
	char *segmentGTFileName = NULL; //Vidovic
	DWORD method = RVLRECOGNITION_METHOD_PSGM;
	//DWORD method = RVLRECOGNITION_METHOD_RF; //VIDOVIC

	DWORD flags = 0x00000000; //VIDOVIC

	CRVLParameterList ParamList;

	CreateParamList(&ParamList,
		&mem0,
		&sceneMeshFileName,
		&sceneSequenceFileName,
		&modelSequenceFileName,
		&modelsInDB,
		&GTFolder,
		&segmentGTFileName,
		method,
		flags);	 //VIDOVIC

	ParamList.LoadParams("RVLRecognitionDemo_all.cfg");

	if (segmentGTFileName == NULL)
	{
		segmentGTFileName = new char[200];
		segmentGTFileName = "C:\\RVL\\segmentGT.txt";
	}

	// Initialize surfel detection

	SurfelGraph surfels;

	surfels.pMem = &mem;

	surfels.CreateParamList(&mem0);

	surfels.ParamList.LoadParams("RVLRecognitionDemo_all.cfg");

	PlanarSurfelDetector surfelDetector;

	surfelDetector.CreateParamList(&mem0);

	surfelDetector.ParamList.LoadParams("RVLRecognitionDemo_all.cfg");

	//VIDOVIC
	//initialize mesh noiser
	MeshNoiser noiser;

	noiser.SetParam(1, 0.05);
	//END VIDOVIC

	// Initialize visualization

	unsigned char SelectionColor[3];

	SelectionColor[0] = 0;
	SelectionColor[1] = 255;
	SelectionColor[2] = 0;

	Visualizer visualizer;

	visualizer.Create();

	if (method == RVLRECOGNITION_METHOD_RF)
	{
		// Initialize recognition.

		RFRecognition recognition;

		recognition.CreateParamList(&mem0);

		recognition.ParamList.LoadParams("RVLRecognitionDemo.cfg");

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

			//VIDOVIC
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


			//VIDOVIC
			RECOG::Hypothesis *pBestHypothesis = NULL;
			float V[3], theta;
			float distance;
			//END VIDOVIC

			mesh.LoadPolyDataFromPLY(sceneMeshFileName);

			recognition.FindObjects(&mesh);

			//VIDOVIC
			recognition.FindBestHypothesis(&pBestHypothesis);

			GetAngleAxis(pBestHypothesis->R, V, theta);
			GetDistance(pBestHypothesis->t, distance);

			FILE *fpHypothesisErrorDebug = NULL;

			fpHypothesisErrorDebug = fopen("C:\\RVL\\Debug\\hypothesisErrorDebug.txt", "w");
			//END VIDOVIC

			FILE *fpInterpretation = fopen("C:\\RVL\\Debug\\interpretation.txt", "w");

			RECOG::Hypothesis *pHypothesis = recognition.sceneInterpretation.pFirst;

			while (pHypothesis)
			{
				RECOG::WriteHypothesis(fpInterpretation, pHypothesis);

				//VIDOVIC
				GetAngleAxis(pHypothesis->R, V, theta);
				GetDistance(pHypothesis->t, distance);

				RECOG::WriteHypothesisError(fpHypothesisErrorDebug, pHypothesis, distance, theta * 180 / PI);
				//END VIDOVIC

				pHypothesis = pHypothesis->pNext;
			}

			fclose(fpInterpretation);
			fclose(fpHypothesisErrorDebug); //VIDOVIC

#endif // RVL_LOAD_SINGLE_MODEL
			//END VIDOVIC

			// Visualization

			surfels.NodeColors(SelectionColor); //VIDOVIC
			recognition.InitDisplay(&visualizer, &mesh);
			recognition.Display();
			visualizer.Run();
		}
	}	// if (method == RVLRECOGNITION_METHOD_RF)
	else if (method == RVLRECOGNITION_METHOD_PSGM)
	{
		// Initialize recognition.

		PSGM recognition;

		recognition.CreateParamList(&mem0);

		recognition.ParamList.LoadParams("RVLRecognitionDemo_all.cfg");

		//recognition.Create();

		recognition.pMem = &mem;

		recognition.pSurfels = &surfels;

		recognition.pSurfelDetector = &surfelDetector;

		if (recognition.mode == RVLRECOGNITION_MODE_TRAINING)
		{
			surfels.NodeColors(SelectionColor);

			recognition.Learn(modelSequenceFileName, &visualizer); //Vidovic
		}
		else if (recognition.mode == RVLRECOGNITION_MODE_RECOGNITION)
		{
			//Eigen::MatrixXf nI = recognition.ConvexTemplatenT();
			//float dI[66];
			//for (int i = 0; i < 66; i++) dI[i] = 1;
			//recognition.RVLPSGInstanceMesh(nI, dI);

			recognition.LoadModelDataBase(); //Vidovic

#ifdef RVLPSGM_ICP
			recognition.LoadModelMeshDB(modelSequenceFileName, true, 0.4);
#endif

			Mesh mesh;

			//Vidovic
			char filePath[200];

			char *CTIFileName = NULL;

			recognition.pTimer = new CRVLTimer;

			FileSequenceLoader sceneSequence;

			sceneSequence.Init(sceneSequenceFileName);

			recognition.pECCVGT->Init(sceneSequence, GTFolder, modelsInDB);

			//recognition.pECCVGT->SaveGTFile("D:\\ARP3D\\TUW_GT.txt");			

			//FILE *fpHypothesisEvaluation = fopen("D:\\ARP3D\\compare_TNM_Valid_TMP.txt", "w");

			//FILE *fpLog = fopen("D:\\ARP3D\\evaluationLog.txt", "w");			

			FILE *fpPoseError = fopen("C:\\RVL\\ExpRez\\poseError.txt", "w");

			FILE *fpnotFirstInfo = fopen("C:\\RVL\\ExpRez\\notFirstInfo.txt", "w");

			FILE *fpnotFirstPoseErr = fopen("C:\\RVL\\ExpRez\\notFirstInfo.txt", "w");

			//recognition.pECCVGT->SaveGTFile("C:\\RVL\\ExpRez\\TUW_GT.txt");

			FILE *fpHypothesisEvaluation = fopen("C:\\RVL\\ExpRez\\compare_TNM_Valid_TMP.txt", "w");

			FILE *fpLog = fopen("C:\\RVL\\ExpRez\\evaluationLog.txt", "w");

			recognition.LoadCompleteSegmentGT(sceneSequence);

			LARGE_INTEGER ctr1, ctr2, freq;
			LARGE_INTEGER ctr1_, ctr2_, freq_;

			while (sceneSequence.GetNextPath(filePath))
			{
				QueryPerformanceCounter((LARGE_INTEGER *)&ctr1);

				printf("Scene %s...\n", filePath);

				recognition.SetSceneFileName(filePath);
				//recognition.InterpretCTIS(&mesh);

#ifdef PSGM_LOAD_CTI_FROM_FILE
				RVLCopyString(filePath, &CTIFileName);

				sprintf(RVLGETFILEEXTENSION(CTIFileName), "cti");

				recognition.LoadCTI(CTIFileName);

				recognition.Match();

				surfels.NodeArray.n = 0;

				recognition.clusters.n = 0;
#else
				mesh.LoadPolyDataFromPLY(filePath);

				mem.Clear();

				recognition.Interpret(&mesh);

				//recognition.SaveMatches();

#ifdef PSGM_RECOGNITION_VISUALIZE_SCENE
				//Visualize currennt scene (close visualizer window by pressing 'q' key)
				surfels.NodeColors(SelectionColor);
				recognition.InitDisplay(&visualizer, &mesh, SelectionColor);
				recognition.Display();
				visualizer.Run();

				visualizer.renderer->RemoveAllViewProps();
#endif
#endif
				//Evaluate CTI match
				//recognition.EvaluateMatchesByScore(fpHypothesisEvaluation, fpLog, 7);

				printf("Scene %s...finished!\n\n", filePath);

				mesh.LoadPolyDataFromPLY(filePath);


				//surfels.NodeColors(SelectionColor);
				
				visualizer.renderer->RemoveAllViewProps();
				recognition.InitDisplay(&visualizer, &mesh, SelectionColor);
				recognition.Display();

				QueryPerformanceCounter((LARGE_INTEGER *)&ctr1_);

#ifdef RVLPSGM_ICP
				//pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_destination(new pcl::PointCloud<pcl::PointXYZINormal>);
				////creating destination cloud
				//cloud_destination->width = mesh.NodeArray.n;
				//cloud_destination->height = 1;
				//cloud_destination->is_dense = false;
				//cloud_destination->points.resize(cloud_destination->width * cloud_destination->height);

				//for (int i = 0; i < mesh.NodeArray.n; i++)
				//{
				//	cloud_destination->points[i].x = mesh.NodeArray.Element[i].P[0];
				//	cloud_destination->points[i].y = mesh.NodeArray.Element[i].P[1];
				//	cloud_destination->points[i].z = mesh.NodeArray.Element[i].P[2];

				//	cloud_destination->points[i].normal_x = mesh.NodeArray.Element[i].N[0];
				//	cloud_destination->points[i].normal_y = mesh.NodeArray.Element[i].N[1];
				//	cloud_destination->points[i].normal_z = mesh.NodeArray.Element[i].N[2];
				//}

				//pcl::search::KdTree<pcl::PointXYZINormal>::Ptr kdtree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZINormal>>((new pcl::search::KdTree<pcl::PointXYZINormal>));
				//kdtree->setInputCloud(cloud_destination); //using this doesn't really improve anything

				//recognition.CalculateICPCost(PCLICP, PCLICPVariants::Point_to_plane, &kdtree);
				GenerateSegmentNeighbourhood(&recognition, 0.1);
				recognition.CalculateNNCost(&visualizer, PCLICP, PCLICPVariants::Point_to_plane);

				//evaluate ICP
				recognition.EvaluateMatchesByScore(fpHypothesisEvaluation, fpLog, fpPoseError, fpnotFirstInfo, fpnotFirstPoseErr, 7, true);
#else
				recognition.EvaluateMatchesByScore(fpHypothesisEvaluation, fpLog, fpPoseError, fpnotFirstInfo, fpnotFirstPoseErr, 7);
#endif
				//recognition.AddModelsToVisualizer(&visualizer, true, PCLICP, PCLICPVariants::Point_to_plane, NULL/*&kdtree*/);
				QueryPerformanceCounter((LARGE_INTEGER *)&ctr2_);
				QueryPerformanceFrequency((LARGE_INTEGER *)&freq_);
				float timevalueICP = (ctr2_.QuadPart - ctr1_.QuadPart) * 1000.0 / freq_.QuadPart;


				QueryPerformanceCounter((LARGE_INTEGER *)&ctr2);
				QueryPerformanceFrequency((LARGE_INTEGER *)&freq);
				float timevalue = (ctr2.QuadPart - ctr1.QuadPart) * 1000.0 / freq.QuadPart;
				std::cout << "Ukupno vrijeme: " << timevalue << std::endl;
				std::cout << "ICP vrijeme: " << timevalueICP << std::endl;
				visualizer.Run();


			}

			RVL_DELETE_ARRAY(CTIFileName);

			RVL_DELETE_ARRAY(recognition.pTimer);

			RVL_DELETE_ARRAY(recognition.segmentGT.Element);

			fclose(fpHypothesisEvaluation);
			fclose(fpLog);

			//END Vidovic
		}	// if (recognition.mode == RVLRECOGNITION_MODE_RECOGNITION)
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

			//char filePath[200];		

			int nM = 35, nSM = 3;

			while (sceneSequence.GetNextPath(filePath))
			{

				printf("Scene %s...\n", filePath);

				mesh.LoadPolyDataFromPLY(filePath);

				recognition.SetSceneFileName(filePath);

				//Alokacija prostora za matcheve - TEMP
				// List of matches
				recognition.SMatch = new RECOG::PSGM_::SegmentMatch[recognition.nDominantClusters*nM*nSM]; //Petra	

				//Sorted matches
				recognition.sortedMatches = new SortIndex<float>[recognition.nDominantClusters*nM*nSM]; //Petra

				recognition.Interpret(&mesh, iScene);
				//recognition.InterpreteCTIS(&mesh);				

				//recognition.LoadCTI("D:\\ARP3D\\ECCV_dataset\\pcd_files\\frame_20111220T111153.549117.cti");

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
			}

			RVL_DELETE_ARRAY(clusterNormalDistributionFileName);

			// Visualization
			surfels.NodeColors(SelectionColor);
			recognition.InitDisplay(&visualizer, &mesh, SelectionColor);
			recognition.Display();
			visualizer.Run();



		}
	}	// if (method == RVLRECOGNITION_METHOD_PSGM)

	// free memory

	if (sceneMeshFileName)
		delete[] sceneMeshFileName;

	//VIDOVIC
	if (sceneSequenceFileName)
		delete[] sceneSequenceFileName;

	if (modelSequenceFileName)
		delete[] modelSequenceFileName;

	
	//if (segmentGTFileName)
	//	delete[] segmentGTFileName;

	//END VIDOVIC

	return 0;
}

