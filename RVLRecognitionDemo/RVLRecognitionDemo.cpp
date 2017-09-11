// RVLRecognitionDemo.cpp : Defines the entry point for the console application.
//
#include <Windows.h>
#include <ctime>
#include <fstream>
//#include "stdafx.h"
#include <vtkAutoInit.h>
//VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
#include "RVLVTK.h"
#include "RVLCore2.h"
#include "Util.h"
#include "Space3DGrid.h"
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
#include "VertexGraph.h"
#include "TG.h"
#include "TGSet.h"
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
//#define RVLRECOGNITION_DEMO_CLASS_ALIGNMENT
#define RVLPSGM_TRANSPARENCY_AND_COLLISION
//#define RVLPSGM_RMSE_CALCULATION
//#define RVLRECOGNITION_DEMO_CLASS_ALIGNMENT

#define RVLRECOGNITION_DEMO_FLAG_SAVE_PLY			0x00000001
#define RVLRECOGNITION_DEMO_FLAG_3D_VISUALIZATION	0x00000002
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
	char **pResultsFolder,
	DWORD &method,
	DWORD &flags
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
	pParamData = pParamList->AddParam("ResultsFolder", RVLPARAM_TYPE_STRING, pResultsFolder);
	pParamData = pParamList->AddParam("SegmentGTFileName", RVLPARAM_TYPE_STRING, pSegmentGTFileName);	//Vidovic
	pParamData = pParamList->AddParam("Recognition.method", RVLPARAM_TYPE_ID, &method);
	pParamList->AddID(pParamData, "PSGM", RVLRECOGNITION_METHOD_PSGM);
	pParamList->AddID(pParamData, "RF", RVLRECOGNITION_METHOD_RF); //VIDOVIC
	pParamData = pParamList->AddParam("Save PLY", RVLPARAM_TYPE_ID, &flags); //VIDOVIC
	pParamList->AddID(pParamData, "yes", RVLRECOGNITION_DEMO_FLAG_SAVE_PLY); //VIDOVIC
	pParamData = pParamList->AddParam("3D Visualization", RVLPARAM_TYPE_ID, &flags);
	pParamList->AddID(pParamData, "yes", RVLRECOGNITION_DEMO_FLAG_3D_VISUALIZATION);
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

	
	int idx = 0;
	for (int i = 0; i <psgm->pMesh->NodeArray.n; i++)
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

void FilterImage(cv::Mat img)
{
	cv::Mat newImg(480, 640, CV_16UC1, cv::Scalar::all(0));
	img.copyTo(newImg);
	float sum = 0;
	float max = 0;
	int no = 0;
	for (int y = 10; y < (img.rows - 10); y++)
	{
		for (int x = 10; x < (img.cols - 10); x++)
		{
			if (img.at<uint16_t>(y, x) > 0)
				continue;
			//inner 
			sum = 0;
			no = 0;
			max = 0;
			for (int v = -1; v < 1; v++)
			{
				for (int u = -1; u < 1; u++)
				{
					if (img.at<uint16_t>(y + v, x + u) == 0)
						continue;
					sum += img.at<uint16_t>(y + v, x + u);
					no++;
					if (img.at<uint16_t>(y + v, x + u) > max)
						max = img.at<uint16_t>(y + v, x + u);
				}
			}
			if (no != 0)
				newImg.at<uint16_t>(y, x) = max;// sum / no;
		}
	}
	newImg.copyTo(img);
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

	char cfgFileName[] = "RVLRecognitionDemo_all.cfg";

	char *sceneMeshFileName = NULL;
	char *sceneSequenceFileName = NULL; //VIDOVIC
	char *modelSequenceFileName = NULL; //VIDOVIC
	char *modelsInDB = NULL; //VIDOVIC
	char *GTFolder = NULL; //VIDOVIC
	char *ResultsFolder = NULL;
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
		&ResultsFolder,
		method,
		flags);	 //VIDOVIC

	ParamList.LoadParams(cfgFileName);

	// Create mesh builder.

	PCLMeshBuilder meshBuilder;

	meshBuilder.CreateParamList(&mem0);

	meshBuilder.ParamList.LoadParams(cfgFileName);

	int w = 640;
	int h = 480;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC(new pcl::PointCloud<pcl::PointXYZRGBA>(w, h));

	meshBuilder.PC = PC;

	if (flags & RVLRECOGNITION_DEMO_FLAG_SAVE_PLY)
	{
		Mesh mesh;

		FileSequenceLoader sceneSequence;
		char filePath[200];

		sceneSequence.Init(sceneSequenceFileName);

		while (sceneSequence.GetNextPath(filePath))
			LoadMesh(&meshBuilder, filePath, &mesh, true);

		if (sceneMeshFileName)
			delete[] sceneMeshFileName;

		if (sceneSequenceFileName)
			delete[] sceneSequenceFileName;

		if (modelSequenceFileName)
			delete[] modelSequenceFileName;

		return 0;
	}

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

		recognition.ParamList.LoadParams(cfgFileName);

		//recognition.Create();

		recognition.pMem = &mem;
		recognition.pMem0 = &mem0;

		recognition.vpMeshBuilder = &meshBuilder;
		recognition.LoadMesh = LoadMesh;

		recognition.pSurfels = &surfels;

		recognition.pSurfelDetector = &surfelDetector;

		recognition.MTGSet.pMem = recognition.pMem0;

		recognition.Init(cfgFileName);

		if (recognition.mode == RVLRECOGNITION_MODE_TRAINING)
			recognition.Learn(modelSequenceFileName, &visualizer); //Vidovic
		else if (recognition.mode == RVLRECOGNITION_MODE_RECOGNITION)
		{
			//Eigen::MatrixXf nI = recognition.ConvexTemplatenT();
			//float dI[66];
			//for (int i = 0; i < 66; i++) dI[i] = 1;
			//recognition.RVLPSGInstanceMesh(nI, dI);

			recognition.LoadModelDataBase(); //Vidovic
			
#ifdef RVLRECOGNITION_DEMO_CLASS_ALIGNMENT
			//Alignment:
			recognition.LoadModelMeshDB(modelSequenceFileName, false, 0.4);
			recognition.ObjectAlignment();
#endif

#ifdef RVLRECOGNITION_DEMO_CLASS_ALIGNMENT
			//Alignment:
			recognition.LoadModelMeshDB(modelSequenceFileName, false, 0.4); //Vidovic merge 20.07.2017 - potrebno izmijeniti poziv funkcije //recognition.LoadModelMeshDB(modelSequenceFileName, &recognition.vtkModelDB, false, 0.4);
			recognition.ObjectAlignment();
#endif

#ifdef RVLPSGM_ICP
			recognition.LoadModelMeshDB(modelSequenceFileName, &recognition.vtkModelDB, true, 0.4);
#endif

			Mesh mesh;

			//Vidovic
			char filePath[200];

			char *CTIFileName = NULL;

			recognition.pTimer = new CRVLTimer;

			FileSequenceLoader sceneSequence;

			sceneSequence.Init(sceneSequenceFileName);

			//recognition.pSurfels->bContactEdgeVertices = true;

			recognition.pECCVGT->Init(sceneSequence, GTFolder, modelsInDB);

			//recognition.pECCVGT->SaveGTFile("D:\\ARP3D\\TUW_GT.txt");			

			//FILE *fpHypothesisEvaluation = fopen("D:\\ARP3D\\compare_TNM_Valid_TMP.txt", "w");

			//FILE *fpLog = fopen("D:\\ARP3D\\evaluationLog.txt", "w");			

			std::string resultsFolderName = std::string(ResultsFolder);

			FILE *fpPoseError = fopen((resultsFolderName + "\\poseError.txt").data(), "w");

			FILE *fpnotFirstInfo = fopen((resultsFolderName + "\\notFirstInfo.txt").data(), "w");

			FILE *fpnotFirstPoseErr = fopen((resultsFolderName + "\\notFirstInfo.txt").data(), "w");

			//recognition.pECCVGT->SaveGTFile("C:\\RVL\\ExpRez\\TUW_GT.txt");

			FILE *fpHypothesisEvaluation = fopen((resultsFolderName + "\\compare_TNM_Valid_TMP.txt").data(), "w");

			FILE *fpLog = fopen((resultsFolderName + "\\evaluationLog.txt").data(), "w");

			FILE *fpRMSE = fopen((resultsFolderName + "\\RMSE.txt").data(), "w");

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
				//mesh.LoadPolyDataFromPLY(filePath);
				LoadMesh(&meshBuilder, filePath, &mesh, false);

#ifdef NEVER 
				//Generate scene depth
				//************************************************************************************************************
				//Vidovic commented on 21.07.2017.
				//because function PSGM::CreateDilatedDepthImage(); is called inside PSGM::FilterHypothesesUsingTransparency()
				//************************************************************************************************************
				//double point[3];
				//int u, v;
				//cv::Mat depth(480, 640, CV_16UC1, cv::Scalar::all(0));
				//for (int i = 0; i < mesh.pPolygonData->GetNumberOfPoints(); i++)
				//{
				//	mesh.pPolygonData->GetPoint(i, point);
				//	if ((point[0] == 0) && (point[1] == 0) && (point[2] == 0))
				//		continue;
				//	v = floor(float(i) / 640);
				//	u = i - v * 640;
				//	depth.at<uint16_t>(v, u) = (uint16_t)(point[2] * 1000); //in milimeters
				//}
				////Postprocessing
				//for (int y = 0; y < depth.rows; y++)
				//{
				//	for (int x = 0; x < depth.cols; x++)
				//	{
				//		if (depth.at<uint16_t>(y, x) == 0)
				//			depth.at<uint16_t>(y, x) = 10000; //in milimeters
				//	}
				//}
				//cv::Mat elementE = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(17, 17));
				//cv::erode(depth, depth, elementE);
				////Set PSGM depth
				//recognition.depthImg = (unsigned short*)depth.data;
				//************************************************************************************************************
				//END Vidovic commented
				//************************************************************************************************************


				/*cv::Mat depthShow(480, 640, CV_8UC1);
				double minVal, maxVal;
				cv::minMaxLoc(depth, &minVal, &maxVal);
				depth.convertTo(depthShow, CV_8U, -255.0f / maxVal, 255.0f);
				cv::imshow("depth image", depthShow);
				cv::waitKey();*/

				///////////TEST/////////
				///*std::fstream fileS("eccv_frame_20111221T142636.413299_depth.txt", std::fstream::out);
				//double point[3];
				//int u, v;
				//for (int i = 0; i < mesh.pPolygonData->GetNumberOfPoints(); i++)
				//{
				//	mesh.pPolygonData->GetPoint(i, point);
				//	if ((point[0] == 0) && (point[1] == 0) && (point[2] == 0))
				//		continue;
				//	v = floor(float(i) / 640);
				//	u = i - v * 640;
				//	fileS << u << " " << v << " " << point[0] << " " << point[1] << " " << point[2] << std::endl;
				//}
				//fileS.close();*/
				////Generate scene depth
				//double point[3];
				//int u, v;
				//cv::Mat origDepth(480, 640, CV_16UC1, cv::Scalar::all(0));
				//for (int i = 0; i < mesh.pPolygonData->GetNumberOfPoints(); i++)
				//{
				//	mesh.pPolygonData->GetPoint(i, point);
				//	if ((point[0] == 0) && (point[1] == 0) && (point[2] == 0))
				//		continue;
				//	v = floor(float(i) / 640);
				//	u = i - v * 640;
				//	origDepth.at<uint16_t>(v, u) = (uint16_t)(point[2] * 1000); //in milimeters
				//}
				//// Initialize VTK.
				//vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
				//vtkSmartPointer<vtkRenderWindow> renWin = vtkSmartPointer<vtkRenderWindow>::New();
				///*vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
				//interactor->SetRenderWindow(renWin);
				//vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
				//interactor->SetInteractorStyle(style);*/
				//renWin->OffScreenRenderingOn(); //OFF-SCREEN RENDERING
				//renWin->AddRenderer(renderer);
				//renWin->SetSize(640, 480); //HARDCODED 640X480 IMAGE

				////adding polydata actor
				//vtkSmartPointer<vtkPolyDataMapper>	map = vtkSmartPointer<vtkPolyDataMapper>::New();
				//map->SetInputData(mesh.pPolygonData);
				//vtkSmartPointer<vtkActor> act = vtkSmartPointer<vtkActor>::New();
				//act->SetMapper(map);
				//renderer->AddActor(act);
				////renWin->Render();
				////interactor->Start();

				////find zbounds
				//double *bounds;
				//mesh.pPolygonData->GetPoints()->ComputeBounds(); //just in case
				//bounds = mesh.pPolygonData->GetPoints()->GetBounds(); // (Xmin, Xmax) = (bounds[0], bounds[1]), (Ymin, Ymax) = (bounds[2], bounds[3]), (Zmin, Zmax) = (bounds[4], bounds[5])
				//if (bounds[4] == 0.0)
				//	bounds[4] = 0.4; //0.4m
				//vtkSmartPointer<vtkCamera> camera = CreateVTKCamera_GenericKinect_1(bounds[4], bounds[5]);
				//cv::Mat bufferDepth;
				//for (int i = 0; i < 10; i++)
				//{
				//	LARGE_INTEGER d_ctr1, d_ctr2, d_freq;
				//	QueryPerformanceCounter((LARGE_INTEGER *)&d_ctr1);
				//	bufferDepth = GenerateVTKDepthImage(renWin, camera, 640, 480);// GenerateVTKDepthImage_Kinect(renWin, bounds[4], bounds[5]);
				//	QueryPerformanceCounter((LARGE_INTEGER *)&d_ctr2);
				//	QueryPerformanceFrequency((LARGE_INTEGER *)&d_freq);
				//	float d_timevalue = (d_ctr2.QuadPart - d_ctr1.QuadPart) * 1000.0 / d_freq.QuadPart;
				//	std::cout << "Depth gen vrijeme: " << d_timevalue << std::endl;
				//}
				////show
				//cv::Mat depthShow(480, 640, CV_8UC1);
				//double minVal, maxVal;
				////FilterImage(bufferDepth);
				//cv::minMaxLoc(bufferDepth, &minVal, &maxVal);
				//bufferDepth.convertTo(depthShow, CV_8U, -255.0f / maxVal, 255.0f);
				//cv::imshow("Rendered depth image", depthShow);
				////cv::imwrite("renderedDepthF.png", bufferDepth);
				///*cv::Mat bufferDepthBlur(480, 640, CV_16UC1, cv::Scalar::all(0));
				//cv::Mat depthShowBlur(480, 640, CV_8UC1);
				//cv::GaussianBlur(bufferDepth, bufferDepthBlur, cv::Size(9, 9), 0, 0);
				//cv::minMaxLoc(bufferDepthBlur, &minVal, &maxVal);
				//bufferDepthBlur.convertTo(depthShowBlur, CV_8U, -255.0f / maxVal, 255.0f);
				//cv::imshow("Rendered depth (blur) image", depthShowBlur);*/
				////Original depth
				//cv::minMaxLoc(origDepth, &minVal, &maxVal);
				//cv::Mat depthOrigShow(480, 640, CV_8UC1);
				//origDepth.convertTo(depthOrigShow, CV_8U, -255.0f / maxVal, 255.0f);
				//cv::imshow("Original depth", depthOrigShow);			
				////cv::imwrite("origDepth.png", origDepth);
				////Dilate original depth
				//cv::Mat elementD = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(20,20));
				//cv::Mat elementE = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(11, 11));
				//cv::Mat origDepth_D(480, 640, CV_16UC1, cv::Scalar::all(0));
				//LARGE_INTEGER d_ctr1, d_ctr2, d_freq;
				//QueryPerformanceCounter((LARGE_INTEGER *)&d_ctr1);
				//for (int y = 0; y < origDepth.rows; y++)
				//{
				//	for (int x = 0; x < origDepth.cols; x++)
				//	{
				//		if (origDepth.at<uint16_t>(y, x) == 0)
				//			origDepth.at<uint16_t>(y, x) = 10000; //in milimeters
				//	}
				//}
				////cv::dilate(origDepth, origDepth_D, elementD);
				//cv::erode(origDepth, origDepth_D, elementE);
				//QueryPerformanceCounter((LARGE_INTEGER *)&d_ctr2);
				//QueryPerformanceFrequency((LARGE_INTEGER *)&d_freq);
				//float d_timevalue = (d_ctr2.QuadPart - d_ctr1.QuadPart) * 1000.0 / d_freq.QuadPart;
				//std::cout << "Dilate vrijeme: " << d_timevalue << std::endl;

				//cv::Mat depthOrigShow_D(480, 640, CV_8UC1);
				//cv::minMaxLoc(origDepth_D, &minVal, &maxVal);
				//origDepth_D.convertTo(depthOrigShow_D, CV_8U, -255.0f / maxVal, 255.0f);
				//cv::imshow("Original depth (dilated)", depthOrigShow_D);

				//////show the difference between rendered depth and original
				////cv::Mat depthDifference(480, 640, CV_16UC1, cv::Scalar::all(0));
				////cv::absdiff(bufferDepth, origDepth, depthDifference);
				////cv::minMaxLoc(depthDifference, &minVal, &maxVal);
				////cv::Mat depthDifferenceShow(480, 640, CV_8UC1);
				////depthDifference.convertTo(depthDifferenceShow, CV_8U, -255.0f / maxVal, 255.0f);
				////cv::imshow("Difference", depthDifferenceShow);

				//cv::waitKey();
				////interactor->Start();
				////
#endif

				mem.Clear();

				recognition.segmentGTLoaded = false;
				recognition.createSegmentGT = false;

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
				if (recognition.problem == RVLRECOGNITION_PROBLEM_CLASSIFICATION)
				{
					// Save the segmentation results to a file.

					char *objectMapFileName = RVLCreateFileName(filePath, ".ply", -1, ".objmap.png");

					cv::Mat objectMask(recognition.pMesh->height, recognition.pMesh->width, CV_8UC1);

					recognition.pObjects->ObjectMapMask(&objectMask);

					cv::imshow("Object mask", objectMask);

					cv::imwrite(objectMapFileName, objectMask);

					delete[] objectMapFileName;

					cv::waitKey();
				}
#endif	// #ifndef PSGM_LOAD_CTI_FROM_FILE
				//Evaluate CTI match
				//recognition.EvaluateMatchesByScore(fpHypothesisEvaluation, fpLog, 7);

				printf("Scene %s...finished!\n\n", filePath);

				//mesh.LoadPolyDataFromPLY(filePath);
				//LoadMesh(&meshBuilder, filePath, &mesh, false);

				if (flags & RVLRECOGNITION_DEMO_FLAG_3D_VISUALIZATION)
				{
					surfels.NodeColors(SelectionColor);
					visualizer.renderer->RemoveAllViewProps();
					recognition.InitDisplay(&visualizer, &mesh, SelectionColor);
					recognition.Display();
				}
				
				////NEW FILKO - TEST COLLISION CONSENSUS
				//std::vector<int> conHyp = recognition.GetHypothesesCollisionConsensus(20);
				//for (int i = 0; i < conHyp.size(); i++)
				//{
				//	recognition.AddOneModelToVisualizer(&visualizer, conHyp.at(i), 0, false, true);
				//}

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
				//recognition.CalculateNNCost(&visualizer, PCLICP, PCLICPVariants::Point_to_plane);
				
				//TEST RVLPSGM_MATCHCTI_MATCH_MATRIX
				recognition.ICP(PCLICP, PCLICPVariants::Point_to_plane);

#ifdef RVLPSGM_TRANSPARENCY_AND_COLLISION
				//Transparency check
				//recognition.CreateScoreMatchMatrixICP();
				recognition.CreateScoreMatchMatrixICP_TMP();
				recognition.FilterHypothesesUsingTransparency(0.15, 10, true);
				recognition.CreateScoreMatchMatrixICP_TMP(); //because of sorting - TEST

				//Colision check
				recognition.noCollisionHypotheses.clear();
				recognition.GetHypothesesCollisionConsensus(&recognition.noCollisionHypotheses, &recognition.scoreMatchMatrixICP, 10);

				//Get transparency and collision consensus
				recognition.GetTransparencyAndCollisionConsensus(&visualizer);

				//Evaluate consesus matches
				float precision, recall;
				recognition.EvaluateConsensusMatches(precision, recall, true);

				//recognition.createVersionTestFile();
				recognition.checkVersionTestFile();
#endif
				
//#ifdef RVLVERSION_170601
//				//evaluate ICP
//				recognition.EvaluateMatchesByScore(fpHypothesisEvaluation, fpLog, fpPoseError, fpnotFirstInfo, fpnotFirstPoseErr, 10, true);
//#endif

#ifdef RVLPSGM_RMSE_CALCULATION
				//Load models without decimation (used for calculatin RMSE)
				recognition.LoadModelMeshDB(modelSequenceFileName, &recognition.vtkRMSEModelDB, false);

				//Calculate RMSE
				recognition.RMSE(fpRMSE, false);
#endif

#else	// #ifndef RVLPSGM_ICP
				//recognition.EvaluateMatchesByScore(fpHypothesisEvaluation, fpLog, fpPoseError, fpnotFirstInfo, fpnotFirstPoseErr, 7);
#endif	// #ifndef RVLPSGM_ICP
				//recognition.AddModelsToVisualizer(&visualizer, true, PCLICP, PCLICPVariants::Point_to_plane, NULL/*&kdtree*/);
				QueryPerformanceCounter((LARGE_INTEGER *)&ctr2_);
				QueryPerformanceFrequency((LARGE_INTEGER *)&freq_);
				float timevalueICP = (ctr2_.QuadPart - ctr1_.QuadPart) * 1000.0 / freq_.QuadPart;


				QueryPerformanceCounter((LARGE_INTEGER *)&ctr2);
				QueryPerformanceFrequency((LARGE_INTEGER *)&freq);
				float timevalue = (ctr2.QuadPart - ctr1.QuadPart) * 1000.0 / freq.QuadPart;
				std::cout << "Ukupno vrijeme: " << timevalue << std::endl;
				std::cout << "ICP vrijeme: " << timevalueICP << std::endl;

				if (flags & RVLRECOGNITION_DEMO_FLAG_3D_VISUALIZATION)
					visualizer.Run();
			}

			RVL_DELETE_ARRAY(CTIFileName);

			RVL_DELETE_ARRAY(recognition.pTimer);

			RVL_DELETE_ARRAY(recognition.segmentGT.Element);

			fclose(fpHypothesisEvaluation);
			fclose(fpLog);
			fclose(fpPoseError);
			fclose(fpRMSE);

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

			///

			int nM = 35, nSM = 3;

			while (sceneSequence.GetNextPath(filePath))
			{

				printf("Scene %s...\n", filePath);

				//mesh.LoadPolyDataFromPLY(filePath);
				LoadMesh(&meshBuilder, filePath, &mesh, false);

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

				// Visualization
				//surfels.NodeColors(SelectionColor);
				////visualizer.renderer->RemoveAllViewProps();								
				//recognition.InitDisplay(&visualizer, &mesh, SelectionColor);
				//recognition.Display();
				//visualizer.Run();
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

// Only for debugging purpose!!!
//
//void RVLMinTest()
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