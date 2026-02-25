// RVLTorusDetectionDemo.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"
#include <Windows.h>
#include <ctime>
#include <fstream>
#include <chrono>
#include <vtkAutoInit.h>
//VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
#include "RVLVTK.h"
#include <vtkTriangle.h>		// Remove after completion of MarchingCubes.
#include "RVLCore2.h"
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
#include <pcl/io/openni2_grabber.h>
#include <pcl/io/openni2/openni2_metadata_wrapper.h>
#include "PCLTools.h"
#include "RGBDCamera.h"
#include "PCLMeshBuilder.h"
#include "MarchingCubes.h"
#include "NeighborhoodTool.h"
#include "VN.h"
#include "VNClass.h"
#include "VNInstance.h"
#include "VNClassifier.h"
#ifdef RVL_ASTRA
#include "astraUtils.h"
#endif
#include "RVLAstra.h"
#include "ObjectDetector.h"

#define RVLTORUSDETECTION_DEMO_FLAG_SAVE_PLY				0x00000001

#define RVLVN_TOROIDALCLUSTERS_VERBOSE //Stanic file

using namespace RVL;
using namespace RECOG;

void CreateParamList(
	CRVLParameterList *pParamList,
	CRVLMem *pMem,
	char **pSceneSequenceFileName,
	char **pResultsFolder,
	DWORD &flags,
	float &tolerance
	)
{
	pParamList->m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	pParamList->Init();

	pParamData = pParamList->AddParam("SceneSequenceFileName", RVLPARAM_TYPE_STRING, pSceneSequenceFileName);
	pParamData = pParamList->AddParam("ResultsFolder", RVLPARAM_TYPE_STRING, pResultsFolder);
	pParamData = pParamList->AddParam("Save PLY", RVLPARAM_TYPE_FLAG, &flags);
	pParamList->AddID(pParamData, "yes", RVLTORUSDETECTION_DEMO_FLAG_SAVE_PLY);
	pParamData = pParamList->AddParam("TorusDetection.tolerance", RVLPARAM_TYPE_FLOAT, &tolerance);
}

int main(int argc, char ** argv)
{
	// Create memory storage.

	CRVLMem mem0;	// permanent memory storage

	mem0.Create(1000000000);

	CRVLMem mem;	// cycle memory storage

	mem.Create(10000000000);

	// Read parameters from a configuration file.

	char cfgSelectionFileName[] = "RVLTorusDetectionDemo.cfg";

	char *cfgFileName = ReadConfigurationFile(cfgSelectionFileName);

	if (cfgFileName == NULL)
	{
		printf("No configuration file name specified!\n");

		return 1;
	}

	char *sceneSequenceFileName = NULL;
	char *ResultsFolder = NULL;
	float tolerance;
	DWORD flags = 0x00000000;

	CRVLParameterList ParamList;

	CreateParamList(&ParamList,
		&mem0,
		&sceneSequenceFileName,
		&ResultsFolder,
		flags,
		tolerance
	);

	ParamList.LoadParams(cfgFileName);

	// Mesh object.

	Mesh mesh;

	// Create mesh builder.

	PCLMeshBuilder meshBuilder;

	meshBuilder.Create(cfgFileName, &mem0);

	int w = 640;
	int h = 480;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC(new pcl::PointCloud<pcl::PointXYZRGBA>(w, h));

	meshBuilder.PC = PC;

	if (flags & RVLTORUSDETECTION_DEMO_FLAG_SAVE_PLY)
	{
		FileSequenceLoader sceneSequence;
		char filePath[200];

		sceneSequence.Init(sceneSequenceFileName);

		while (sceneSequence.GetNextPath(filePath))
			LoadMesh(&meshBuilder, filePath, &mesh, true);

		if (sceneSequenceFileName)
			delete[] sceneSequenceFileName;

		return 0;
	}

	// Initialize surfel detection

	SurfelGraph surfels;

	surfels.pMem = &mem;

	surfels.CreateParamList(&mem0);

	surfels.ParamList.LoadParams(cfgFileName);

	PlanarSurfelDetector surfelDetector;

	surfelDetector.CreateParamList(&mem0);

	surfelDetector.ParamList.LoadParams(cfgFileName);

	// Create VN classifier.

	VNClassifier classifier;

	classifier.pMem0 = &mem0;
	classifier.pMem = &mem;
	classifier.vpMeshBuilder = &meshBuilder;
	classifier.LoadMesh = LoadMesh;
	classifier.pSurfels = &surfels;
	classifier.pSurfelDetector = &surfelDetector;

	classifier.Create(cfgFileName);

	classifier.camera.fu = meshBuilder.camera.depthFu;
	classifier.camera.fv = meshBuilder.camera.depthFv;
	classifier.camera.uc = meshBuilder.camera.depthUc;
	classifier.camera.vc = meshBuilder.camera.depthVc;

	// Initialize visualization.

	unsigned char SelectionColor[3];

	SelectionColor[0] = 0;
	SelectionColor[1] = 255;
	SelectionColor[2] = 0;

	Visualizer visualizer;

	visualizer.Create();

	classifier.visualizationData.pVisualizer = &visualizer;

	/// Detect toruses in all images listed in sceneSequence.

	FileSequenceLoader sceneSequence;

	sceneSequence.Init(sceneSequenceFileName);

	float axis[] = { 0.0f, 0.0f, 1.0f };

	Array<float> alphaArray;

	alphaArray.n = 16;

	alphaArray.Element = new float[16];

	float dAlpha = 2 * PI / 16.0f;

	int iAlpha;

	for (iAlpha = 0; iAlpha < 16; iAlpha++)
		alphaArray.Element[iAlpha] = dAlpha * (float)iAlpha;

	Array<float> betaArray;

	betaArray.n = 7;

	betaArray.Element = new float[7];

	float dBeta = PI / 7.0f;

	int iBeta;

	for (iBeta = 0; iBeta < 7; iBeta++)
		betaArray.Element[iBeta] = dBeta * (float)(iBeta + 1);

	char filePath[200];
	VN VNModelTemplate;

	while (sceneSequence.GetNextPath(filePath))
	{
		printf("Scene: %s:\n", filePath);
		classifier.alignment.SetSceneFileName(filePath);
		classifier.SetSceneFileName(filePath);

		// Load mesh.

		LoadMesh(&meshBuilder, filePath, &mesh, false);

		// Reset memory storage.

		mem.Clear();

		// Segment mesh to surfels.				

		surfels.Init(&mesh);

		surfelDetector.Init(&mesh, &surfels, &mem);

		printf("Segmentation to surfels... ");

		surfelDetector.Segment(&mesh, &surfels);

		printf("completed.\n");
		printf("No. of surfels = %d\n", surfels.NodeArray.n);

		surfels.DetectVertices(&mesh);

		surfels.DetectOcclusionVertices(&mesh, classifier.camera);

		// Cluster surfels into convex and concave surfaces.

		bool bConcavity, bTorus;

		bConcavity = true;
		bTorus = true;

		RVL_DELETE_ARRAY(classifier.SClusters.Element);

		RVL_DELETE_ARRAY(classifier.clusterMap);

		classifier.clusterMap = new int[surfels.NodeArray.n];

		int nSCClusters, nSUClusters;

		int *clusterMapAll = new int[surfels.NodeArray.n];

		RECOG::PSGM_::ConvexAndConcaveClusters(&mesh, &surfels, &surfelDetector, &(classifier.convexClustering), &(classifier.concaveClustering),
			classifier.SClusters, nSCClusters, nSUClusters, classifier.convexClustering.minClusterSize, classifier.clusterMap, classifier.maxnSCClusters, classifier.maxnSUClusters, 
			bConcavity, (bTorus ? classifier.maxnSTClusters : 0),
			classifier.visualizationData.bVisualizeConvexClusters, classifier.visualizationData.bVisualizeConcaveClusters, classifier.visualizationData.bVisualizeSurfels);

		RECOG::CreateObjectGraphFromSceneClusters(classifier.SClusters, classifier.pObjects);

		classifier.pObjects->GetVertices();

		// Initialize VNClassifier. 

		classifier.Init(&mesh);

		/// Create supersegments. 

		classifier.CreateSuperSegments();

		if (classifier.superSegments.n == 0)
		{
			printf("No valid supersegments!\n");

			classifier.FreeTmpMem();

			return 0;
		}

		printf("No. of supersegments = %d\n", classifier.superSegments.n);

#ifdef RVLVN_TOROIDALCLUSTERS_VERBOSE

		FILE *pFileTorusInfo = NULL;
		pFileTorusInfo = fopen((std::string(ResultsFolder) + "\\torusinfo.txt").data(), "a+");
		fprintf(pFileTorusInfo, "Scene:%s\n\n", filePath);
		fclose(pFileTorusInfo);
#endif
		
		// P <- surfel vertices

		float *P = new float[3 * surfels.vertexArray.n];

		//float *P_ = P;

		//int iVertex;
		//SURFEL::Vertex *pVertex;

		//for (iVertex = 0; iVertex < surfels.vertexArray.n; iVertex++, P_ += 3)
		//{
		//	pVertex = surfels.vertexArray.Element[iVertex];

		//	RVLCOPY3VECTOR(pVertex->P, P_);
		//}

		// N <- surfel normals

		float *N = new float[3 * surfels.NodeArray.n];

		//float *N_ = N;

		//int iSurfel;
		//Surfel *pSurfel;

		//for (iSurfel = 0; iSurfel < surfels.NodeArray.n; iSurfel++, N_ += 3)
		//{
		//	pSurfel = surfels.NodeArray.Element + iSurfel;

		//	RVLCOPY3VECTOR(pSurfel->N, N_);
		//}

		// Detect toruses.

		Array<RECOG::VN_::Torus *> STClusters;

		float axis_T[3];
		RVLSET3VECTOR(axis_T, 0, 0, 1);
		
		int iSuperSegment, nVertices, i,j;
		float MaxArea, *N_S, d_S; // axis hardkodiran ranije stoga axis_S
		bool condition;
		float *axis_S;

		//Mesh convexHull;

		CRVLMem memConvexHull;
		memConvexHull.Create(1e7);

		std::vector<RVL::Point> MidPointCH;

		QList<VN_::Torus> torusList;

		QList<VN_::Torus> *pTorusList = &torusList;

		RVLQLIST_INIT(pTorusList);

		Array<VN_::Torus *> TClusters;

		TClusters.n = 0;


		for (iSuperSegment = 0; iSuperSegment < classifier.superSegments.n; iSuperSegment++)
		{

			RECOG::VN_::SuperSegment *pSuperSegment;
			pSuperSegment = classifier.superSegments.Element[iSuperSegment];
			
			Mesh convexHull;

			RVL::Array2D<float> convexHullVertices;

			nVertices = pSuperSegment->iVertexArray.n; //postavljanje nVertices u broj verteksa klastera


			convexHullVertices.w = 3;
			convexHullVertices.h = nVertices;
			convexHullVertices.Element = (float*)malloc(convexHullVertices.w * convexHullVertices.h * sizeof(float));

			for (i = 0; i < nVertices; i++)
			{
				float* pointSource = surfels.vertexArray.Element[pSuperSegment->iVertexArray.Element[i]]->P;
				float* pointTarget = convexHullVertices.Element + 3 * i;
				RVLCOPY3VECTOR(pointSource, pointTarget);
			}

			convexHull.ConvexHull(convexHullVertices, &memConvexHull);



			MESH::Face *pFace;

			pFace = convexHull.faces.Element[convexHull.iVisibleFaces.Element[0]];
			//postavi nulti tj. prvi element kao max, pa petlja kreće od 1
			MaxArea = pFace->Area;
			axis_S = pFace->N;

			for (i = 1; i < convexHull.iVisibleFaces.n; i++)
			{
				pFace = convexHull.faces.Element[convexHull.iVisibleFaces.Element[i]];

				if (MaxArea < pFace->Area) {
					MaxArea = pFace->Area;
					axis_S = pFace->N;
				}
			}
			//u axis_S se nalazi normala najveće plohe conv.hulla koja ce biti predana ToroidalClusters

			//1. Lokalni k.s. supersegmenta

			float RST[9]; // Rotacijska matrica iz k.s. scene u k.s. torusa
			// Osi k.s. torusa (T) prikazane u k.s. scene (S) predstavljaju retke rotacijske matrice RST.
			float *XTS = RST;
			float *YTS = RST + 3;
			float *ZTS = RST + 6;						
			RVLCOPY3VECTOR(axis_S, ZTS);				// ZTS je istovjetna osi torusa.
			int j, k;
			float fTmp;
			RVLORTHOGONAL3(ZTS, XTS, i, j, k, fTmp);	// XTS je proizvoljni vektor okomit na ZTS.
			RVLCROSSPRODUCT3(ZTS, XTS, YTS);			// YTS = ZTS x XTS

			//2. Transformacija vertexa k.s. scene u k.s. torusa

			//float *P = new float[3 * surfels.vertexArray.n];

			float *P_ = P;

			int iVertex;
			SURFEL::Vertex *pVertex;

			for (iVertex = 0; iVertex < surfels.vertexArray.n; iVertex++, P_ += 3)
			{
				pVertex = surfels.vertexArray.Element[iVertex];

				RVLMULMX3X3VECT(RST, pVertex->P, P_);
			}

			//3. Transformacija svih normala surfela iz k.s. scene u k.s. torusa

			//float *N = new float[3 * surfels.NodeArray.n];

			float *N_ = N;

			int iSurfel;
			Surfel *pSurfel;

			for (iSurfel = 0; iSurfel < surfels.NodeArray.n; iSurfel++, N_ += 3)
			{
				pSurfel = surfels.NodeArray.Element + iSurfel;

				RVLMULMX3X3VECT(RST, pSurfel->N, N_);
			}

			bool* vertexCH = (bool*)calloc(surfels.vertexArray.n, sizeof(bool));

			for (i = 0; i < surfels.vertexArray.n; i++)
			{
				condition = true;

				float* Point_ = surfels.vertexArray.Element[i]->P;

				//shvaceno kao : ako za svaki od faceova, uvijet <= 0.01 vrijedi, tocka(vertex) se nalazi unutar conv.hulla
				//for (j = 0; j < convexHull.iVisibleFaces.n; j++)
				//{
				//	pFace = convexHull.faces.Element[convexHull.iVisibleFaces.Element[j]];

				for (j = 0; j < convexHull.faces.n; j++)
				{
					pFace = convexHull.faces.Element[j];

					N_S = pFace->N;
					d_S = pFace->d;

					if (RVLDOTPRODUCT3(N_S, Point_) - d_S > 0.01) {
						condition = false;
						break; //ideja : prestanak izvodenja J petlje jer se točka ne nalazi unutar conv. hulla
					}
				}

				if (condition)
					*(vertexCH + i) = true; //ako petlja nije prekinuta, promjeni polje u 1
			}

			/*
			if (iCluster == 8)
			{
				Visualizer vis;

				vis.Create();

				vis.SetMesh(&mesh);

				convexHull.CreateVTKPolyData();

				vis.AddMesh(convexHull.pPolygonData);

				vis.Run();
			}
			*/

			int IsTorusFound = TClusters.n;
			VNModelTemplate.ToroidalClusters2(&mesh, P, N, &surfels, &surfelDetector, axis_T, alphaArray, betaArray, tolerance, pTorusList, &mem, vertexCH, TClusters.n, iSuperSegment);

			// U label zabilježiti ID zadnjeg torusa koji će koristiti tu srednju točku / axis_S

			if (IsTorusFound != TClusters.n){
				float P_0 = 0, P_1 = 0, P_2 = 0, noOfPts;
				RVL::Point MidPoint;

				noOfPts = (float)convexHull.NodeArray.n;

				MidPoint.label = TClusters.n;

				for (i = 0; i < noOfPts; i++){

					P_0 = P_0 + convexHull.NodeArray.Element[i].P[0];
					P_1 = P_1 + convexHull.NodeArray.Element[i].P[1];
					P_2 = P_2 + convexHull.NodeArray.Element[i].P[2];

				}
				MidPoint.P[0] = P_0 / noOfPts;
				MidPoint.P[1] = P_1 / noOfPts;
				MidPoint.P[2] = P_2 / noOfPts;

				MidPoint.N[0] = *(axis_S + 0) / 10 + MidPoint.P[0];
				MidPoint.N[1] = *(axis_S + 1) / 10 + MidPoint.P[1];
				MidPoint.N[2] = *(axis_S + 2) / 10 + MidPoint.P[2];

				MidPointCH.push_back(MidPoint);


			}
			
			free(vertexCH);
			free(convexHullVertices.Element);
		}



		TClusters.Element = new VN_::Torus *[TClusters.n];

		QLIST::CreatePtrArray<VN_::Torus>(pTorusList, &TClusters);

		printf("No. of toruses = %d\n", TClusters.n);

		// Visualization.
		TorusVisualization(&mesh, &surfels, &surfelDetector, torusList, MidPointCH);

		// Free memory.
		delete[] TClusters.Element;



	}


	classifier.FreeTmpMem();

	delete[] alphaArray.Element;
	delete[] betaArray.Element;

	///

	return 0;
}

