#include <algorithm>
#include "RVLCore2.h"
#include "RVLVTK.h"
#include <vtkLine.h>
#include "Util.h"
//#include "Space3DGrid.h"
#include "Graph.h"
#include "Mesh.h"
#//include "MSTree.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
#ifdef MATEJA
#include "ReconstructionEval.h"
#endif
#include "SurfelGraph.h"
//#include "ObjectGraph.h"
//#include "PlanarSurfelDetector.h"
//#include "RVLRecognition.h"
//#include "PSGMCommon.h"
//#include "CTISet.h"
//#include "VertexGraph.h"
//#include "TG.h"
//#include "TGSet.h"
//#include "PSGM.h"
//#include "ObjectDetector.h"
//#ifdef RVLLINUX
//#include <Eigen/Eigenvalues>
//#else
//#include <Eigen\Eigenvalues>
//#endif
//#include "NeighborhoodTool.h"
//#include "VN.h"
//#include "VNInstance.h"
//#include "VNClassifier.h"
#include "ProjectionCube.h"
#include "View.h"
#include "VNLocalizer.h"


#define RVLVNLOCALIZER_POINTS_DEBUG
#define RVLVNLOCALIZER_REGISTRATION_V2
//#define RVLVNLOCALIZER_LOG_RESULTS_SCENESEQUENCE

//#define RVLVNLOCALIZER_MATCH_DEBUG



using namespace RVL;
using namespace LOCAL;

VNLocalizer::VNLocalizer()
{
	minSurfelSize = 1000;
	rotErrorBound = 3.0f * DEG2RAD;
}


VNLocalizer::~VNLocalizer()
{
}



void VNLocalizer::CreateParamList()
{
	paramList.m_pMem = pMem0;

	RVLPARAM_DATA *pParamData;

	paramList.Init();

	pParamData = paramList.AddParam("Localization.mode", RVLPARAM_TYPE_ID, &mode);
	paramList.AddID(pParamData, "TRAINING", RVLLOCALIZATION_MODE_TRAINING);
	paramList.AddID(pParamData, "LOCALIZATION", RVLLOCALIZATION_MODE_LOCALIZATION);

	// dodati localization.camerqaFileName
	//pParamData = paramList.AddParam("Localization.cameraFileName", RVLPARAM_TYPE_STRING, pCameraFileName);
	
}

void VNLocalizer::Create(char *cfgFileName)
{
	// Set parameters.

	CreateParamList();

	paramList.LoadParams(cfgFileName);

	// Image properties
	image_width = camera.w;		// 320
	image_height = camera.h;	// 240

	// Sample image to cells
	// Set size of one cell
	cell_size_width = 16;
	cell_size_height = cell_size_width;

	// Get total number of cells in image
	noColumnsImage = image_width / cell_size_width;		// = 20;
	noRowsImage = image_height / cell_size_height;		// = 15;

	GetCenteralPointsOfImageCells(centralPointsImage);
}

void VNLocalizer::Clear()
{
	int iView;

	for (iView = 0; iView < Views.size(); iView++)
		delete Views[iView];
}

void VNLocalizer::GetCenteralPointsOfImageCells(Array<Point> &centralPoints_array)
{
	// Params.

	int border = 2 * (int)ceil(2.0f * camera.fu * rotErrorBound);

	//
	
	// Get center of a cell 
	int u_init = (int)(floor(cell_size_width / 2)) - 1;		// = 7;
	int v_init = (int)(floor(cell_size_height / 2)) - 1;	// = 7;

	int v = v_init;

	// Initialize Array 
	centralPoints_array.n = noRowsImage * noColumnsImage;;
	centralPoints_array.Element = new Point[centralPoints_array.n];

	centralPoints_array.n = 0;

	int ptIndex = 0;

	for (int cells_row = 0; cells_row < noRowsImage; cells_row++)
	{
		int u = u_init;

		for (int cells_column = 0; cells_column < noColumnsImage; cells_column++)
		{
			// Go to next cell in a row
			u = u + cell_size_width;

			if (u < border || u >= image_width - border || v < border || v >= image_height - border)
				continue;

			

			// pretvori ovo u P[3]
			Point *pCurrentPoint = centralPoints_array.Element + ptIndex;
			float P[3];

			pCurrentPoint->P[0] = (u - camera.uc) / camera.fu;
			pCurrentPoint->P[1] = (v - camera.vc) / camera.fv;
			pCurrentPoint->P[2] = 1;

			//provjeriti je li point valid preko mesha??

			//vrati popis
			ptIndex++;
			centralPoints_array.n++;
		}

		// Go to next row
		v = v + cell_size_height;
	}
}


void VNLocalizer::GetCenteralPointsFromView(View *view)
{
	// Get center of a cell 
	int u_init = (int)(floor(cell_size_width / 2)) - 1;		// = 7;
	int v_init = (int)(floor(cell_size_height / 2)) - 1;	// = 7;

	int v = v_init;

	// Initialize Array 
	view->centralPoints_array.n = noRowsImage * noColumnsImage;
	view->centralPoints_array.Element = new Point[view->centralPoints_array.n];

	int ptIndex = 0;

	for (int cells_row = 0; cells_row < noRowsImage; cells_row++)
	{
		int u = u_init;

		for (int cells_column = 0; cells_column < noColumnsImage; cells_column++)
		{
			//izracunaj sto treba

			// Go to next cell in a row
			u = u + cells_column * noColumnsImage;

			// pretvori ovo u P[3]
			Point *pCurrentPoint = view->centralPoints_array.Element + ptIndex;
			float P[3];

			pCurrentPoint->P[0] = (u - camera.uc) / camera.fu;
			pCurrentPoint->P[1] = (v - camera.vc) / camera.fv;
			pCurrentPoint->P[2] = 1;

			//provjeriti je li point valid preko mesha??

			//vrati popis
			ptIndex++;
		}

		// Go to next row
		v = v + cells_row * noRowsImage;
	}

}

#ifndef AR20
//Vizualizacija surfela
//void VNLocalizer::UpdateMap(Mesh *pMesh, CameraView *pCameraView, char *fileName)	// v1 - SUNCG
void VNLocalizer::UpdateMap(Mesh *pMesh, CameraView cameraView, char *fileName)
{
	// Visualization

	Visualizer *pVisualizer = NULL;

	bool bVisualizeSurfels = true;				//change to "true" to create visualization of surfels

	uchar SelectionColor[] = { 0, 255, 0 };

	bool bVisualizerCallbackFunctionsDefined;

	//if (bVisualizeConvexClusters || bVisualizeConcaveClusters || bVisualizeSurfels || bVisualizeAllClusters)
	if (bVisualizeSurfels)
	{
		pVisualizer = new Visualizer;

		pVisualizer->Create();

		bVisualizerCallbackFunctionsDefined = pSurfels->DisplayData.bCallbackFunctionsDefined;

		pSurfels->DisplayData.bCallbackFunctionsDefined = false;
	}

	// Detect surfels.

	pSurfels->Init(pMesh);

	pSurfelDetector->Init(pMesh, pSurfels, pMem);

	printf("Segmentation to surfels...");

	pSurfelDetector->Segment(pMesh, pSurfels);

	printf("completed.\n");

	int nSurfels = pSurfels->NodeArray.n;

	printf("No. of surfels = %d\n", nSurfels);

	pSurfels->DetectVertices(pMesh);

	pSurfels->DetectOcclusionVertices(pMesh, camera);
	
	/////////////
	// new, Mateja
	//
	
	// Create view.
	View* view = new View;

	//view->Create(fileName, pMesh, pSurfels, pCameraView); //SUNCG - v1
	view->Create(fileName, pMesh, pSurfels, cameraView);

	//view.Copy
	Views.push_back(view);  //Adding current view to previus views.


	// Creating txt file with list of detected surfels in a view
	FILE* pSurfelsFile = NULL;
	char *viewFileNeme = RVLCreateFileName(fileName, ".ply", -1, "_info.txt");
	pSurfelsFile = fopen(viewFileNeme, "w");

	if (pSurfelsFile == NULL)
	{
		printf("Surfels file could not be created.");
	}

	char *cameraFileName = RVLCreateFileName(fileName, ".ply", -1, "_camera.txt");
	FILE* pCameraFile = NULL;
	pCameraFile = fopen(cameraFileName, "w");
	if (pCameraFile == NULL)
	{
		printf("Camera file could not be created.");
	}
	
	// Creating txt file with list of points in a view
	char *pointsFileName = RVLCreateFileName(fileName, ".ply", -1, "_pts.txt");
	FILE* pPointsFile = NULL;
	pPointsFile = fopen(pointsFileName, "w");

	if (pPointsFile == NULL)
	{
		printf("Points file could not be created.");
	}
	
	// Creating txt file with list of surfels points surfels in a view
	char *surfelPtsFileName = RVLCreateFileName(fileName, ".ply", -1, "_SurfelPts.txt");
	FILE* pSurfelPtsFile = NULL;
	pSurfelPtsFile = fopen(surfelPtsFileName, "w");

	
	// Writing surfel info (Surfel ID, R, t, N, d, r1 & r2) and surfel points in files
	if (pSurfelPtsFile == NULL)
	{
		printf("Surfels Points file could not be created.");
	}
	
	if (pSurfelPtsFile != NULL && pSurfelsFile != NULL)
	{
		//for (std::vector<View*>::iterator currentView = Views.begin(); currentView != Views.end(); currentView++)
		
		for (int surfelIndex = 0; surfelIndex < view->surfels.NodeArray.n; surfelIndex++)
		{
			Surfel *pCurrentSurfel = view->surfels.NodeArray.Element + surfelIndex;

			if (pCurrentSurfel->size < minSurfelSize)
				continue;

		
			QList<QLIST::Index2> *pPtListSrc = &(pCurrentSurfel->PtList);

			QLIST::Index2 *pPtIdxSrc = pPtListSrc->pFirst;	// pointer to the first element of pPtListSrc

			while (pPtIdxSrc)
			{
				fprintf(pSurfelPtsFile, "%d %d\n", surfelIndex, pPtIdxSrc->Idx);
				pPtIdxSrc = pPtIdxSrc->pNext;
			}

			fprintf(pSurfelsFile, "%d ", surfelIndex);

			for (int i = 0; i < 9; ++i)
			{
				fprintf(pSurfelsFile, "%f ", pCurrentSurfel->R[i]);
			}

			for (int i = 0; i < 3; ++i)
			{
				fprintf(pSurfelsFile, "%f ", pCurrentSurfel->P[i]);
			}
			for (int i = 0; i < 3; ++i)
			{
				fprintf(pSurfelsFile, "%f ", pCurrentSurfel->N[i]);		//Redudantno, to je zapravo treci stupac iz matirce R
			}

			fprintf(pSurfelsFile, "%f ", pCurrentSurfel->d);

			fprintf(pSurfelsFile, "%f %f", pCurrentSurfel->r1, pCurrentSurfel->r2);
			fprintf(pSurfelsFile, "\n");
		}
		fclose(pSurfelPtsFile);
		fclose(pSurfelsFile);


// Writing all points in a view to a file.
#ifdef RVLVNLOCALIZER_POINTS_DEBUG
		for (int pointIndex = 0; pointIndex < view->mesh.NodeArray.n; pointIndex++)
		{
			Point *pCurrentPoint = view->mesh.NodeArray.Element + pointIndex;

			fprintf(pPointsFile, "%d ", pointIndex);
			for (int i = 0; i < 3; i++)
			{
				fprintf(pPointsFile, "%f ", pCurrentPoint->P[i]);
			}
			fprintf(pPointsFile, "\n");
		}
		fclose(pPointsFile);
#endif

		for (int i = 0; i < 9 ; i++)
		{
			fprintf(pCameraFile, "%f ", view->cameraView.R[i]);
		}

		for (int i = 0; i < 3; i++)
		{
			fprintf(pCameraFile, "%f ", view->cameraView.t[i]);
		}
		fclose(pCameraFile);

	}

	delete[] viewFileNeme;

	
	int numberOfViews = Views.size();


	// Searching for corresponding surfels between all views and writing them in file.
	//		!!!		Ovo vidjeti  treba li optimizirati? tj, da se pozove tek kad su uicatni svi viewi?	//
	if (Views.size() == 3)
	{
		//Write detected Surfel Pairs to the file
		for (int view_ID = 0; view_ID < numberOfViews; view_ID++)
		{
			printf("/n Writing surfel pairs from the view #%d to the file...", view_ID);
			char *fileName4 = RVLCreateFileName(Views[view_ID]->fileName, ".ply", -1, "_SurfelPairs.txt");
			FILE* pSurfelPairsFile = NULL;
			pSurfelPairsFile = fopen(fileName4, "w");

			if (pSurfelPairsFile == NULL)
			{
				printf("Surfel pairs file could not be created.");
			}
			else
			{

				int pairNo = 1;
				for (int i = 0; i < Views[view_ID]->surfelPairs_list.size(); i++)
				{
					fprintf(pSurfelPairsFile, "%d ", pairNo);
					fprintf(pSurfelPairsFile, "%d %d", Views[view_ID]->surfelPairs_list[i]->S1_ID, Views[view_ID]->surfelPairs_list[i]->S2_ID);
					fprintf(pSurfelPairsFile, "\n");
					pairNo++;
				}
			}

			fclose(pSurfelPairsFile);
			printf(" completed.");
		}
		
		
		//Detecting surfels correspondence between views
		for (int viewID_1 = 0; viewID_1 < numberOfViews - 1; viewID_1++)
		{
			
			for (int viewID_2 = viewID_1 + 1; viewID_2 < numberOfViews; viewID_2++)
			{
				
				printf("\nSurfel correspondence between view #%d and #%d...", viewID_1 + 1, viewID_2 + 1);
				Associate(Views[viewID_1], Views[viewID_2], viewID_1, viewID_2);
				printf("completed.\n");

				//  Creating surfel pairs in first view then searching for coresponding surfel pairs in other view.
				printf("\nSurfel pairs correspondence between view #%d and #%d...", viewID_1 + 1, viewID_2 + 1);
				CorrespondingSurfelPairs(Views[viewID_1], Views[viewID_2], viewID_1, viewID_2);
				printf("completed.\n");

				// Determining the poses of the views from which the considered point clouds are acquired w.r.t. a common reference frame S0.
				//x = MultiplePoinCloudReg();
			}

			
			//Write detected corresponding surfels in file
			int NumberOfCrspSurfels = crspSurfels.size();

			char *fileName = RVLCreateFileName(Views[viewID_1]->fileName, ".ply", -1, "_crspSurfels.txt");
			FILE* pCorrespondingSurfelsFile = NULL;
			pCorrespondingSurfelsFile = fopen(fileName, "w");

			if (pCorrespondingSurfelsFile == NULL)
			{
				printf("Corresponding Surfels file could not be created.");
			}
			else
			{
				std::vector<struct CrspSurfels*>::iterator it;

				for (it = crspSurfels.begin(); it != crspSurfels.end(); it++)
				{
					CrspSurfels* currentSurfelPair = *it;
					fprintf(pCorrespondingSurfelsFile, "%d %d %d %d", currentSurfelPair->viewID_1, currentSurfelPair->surfelID_vw1, currentSurfelPair->viewID_2, currentSurfelPair->surfelID_vw2);
					fprintf(pCorrespondingSurfelsFile, "\n");
				}
			}

			fclose(pCorrespondingSurfelsFile);

			//Write corresponding surfel pairs to file.
			char *fileName2 = RVLCreateFileName(Views[viewID_1]->fileName, ".ply", -1, "_crspSurfelPairs.txt");
			FILE* pCrspSurfelPairsFile = NULL;
			pCrspSurfelPairsFile = fopen(fileName2, "w");		
			
			if (pCrspSurfelPairsFile == NULL)
			{
				printf("Generated coresponding surfel pairs file could not be created.");
			}
			else
			{

				//std::vector<struct CrspSurfelPairs*>::iterator it_crspSurfelPairs;
				for (int crspSurfelPairs_ID = 0; crspSurfelPairs_ID < crspPairs.size(); crspSurfelPairs_ID++)
				{
					fprintf(pCrspSurfelPairsFile, "%d %d %d %d %d %d %.4f", crspPairs[crspSurfelPairs_ID]->viewID_1, crspPairs[crspSurfelPairs_ID]->S1_ID_vw1, crspPairs[crspSurfelPairs_ID]->S2_ID_vw1, crspPairs[crspSurfelPairs_ID]->viewID_2, crspPairs[crspSurfelPairs_ID]->S1_ID_vw2, crspPairs[crspSurfelPairs_ID]->S2_ID_vw2, crspPairs[crspSurfelPairs_ID]->sigma_sum);
					fprintf(pCrspSurfelPairsFile, "\n");
				}
			}
			fclose(pCrspSurfelPairsFile);

			//Write the best corresponding surfel pairs for view viewID_1.
			char *fileName3 = RVLCreateFileName(Views[viewID_1]->fileName, ".ply", -1, "_BestCrspPairs.txt");
			FILE* pBestPairsFile = NULL;
			pBestPairsFile = fopen(fileName3, "w");

			if (pBestPairsFile == NULL)
			{
				printf("Best coresponding surfel pairs file could not be created.");
			}
			else
			{
				//std::vector<struct CrspSurfelPairs*>::iterator it_crspSurfelPairs;
				for (int i = 0; i < crspPairs_best.size(); i++)
				{
					fprintf(pBestPairsFile, "%d %d %d %d %d %d %.4f", crspPairs_best[i]->viewID_1, crspPairs_best[i]->S1_ID_vw1, crspPairs_best[i]->S2_ID_vw1, crspPairs_best[i]->viewID_2, crspPairs_best[i]->S1_ID_vw2, crspPairs_best[i]->S2_ID_vw2, crspPairs_best[i]->sigma_sum);
					fprintf(pBestPairsFile, "\n");
				}
			}
			fclose(pBestPairsFile);



			crspSurfels.clear();
			crspPairs.clear();
			crspPairs_best.clear();

			

		}

	}


	//
	// end Mateja
	/////////////
 
	
	// Surfel visualization.

	if (bVisualizeSurfels)
	{
		pSurfels->NodeColors(SelectionColor);

		pSurfels->InitDisplay(pVisualizer, pMesh, pSurfelDetector);

		pSurfels->Display(pVisualizer, pMesh);

		pSurfels->DisplayEdgeFeatures();

		pVisualizer->Run();

		pVisualizer->renderer->RemoveAllViewProps();
	}


	// Save mesh to a PLY file.
	pMesh->SavePolyDataToPLY("surfels.ply");


	// Delete visualizer.
	if (pVisualizer)
	{
		delete pVisualizer;

		pSurfels->DisplayData.bCallbackFunctionsDefined = bVisualizerCallbackFunctionsDefined;
	}
}

////////////
/// mateja

//Associating pair of surfels from two views if surfel seen in the second view matches surfel seen in the first view 
void VNLocalizer::Associate(View* view1, View* view2, int viewID_1, int viewID_2) //argumenti
{
	Surfel *pSurfel1, *pSurfel2;

	//Searching for pairs of corresponding surfers between two views
	for (int SurfelID_vw1 = 0; SurfelID_vw1 < view1->surfels.NodeArray.n; SurfelID_vw1++)					//searching through every surfel in the first view										
	{
		pSurfel1 = &view1->surfels.NodeArray.Element[SurfelID_vw1];
		if (pSurfel1->size < minSurfelSize)
			continue;

		for (int SurfelID_vw2 = 0; SurfelID_vw2 < view2->surfels.NodeArray.n; SurfelID_vw2++)				//searching through every surfel in the second view
		{			

			pSurfel2 = &view2->surfels.NodeArray.Element[SurfelID_vw2];
			if (pSurfel2->size < minSurfelSize)
				continue;

			//CameraView *pCameraView1, *pCameraView2;		//SUNCG v-1
			//pCameraView1 = view1->pCameraView;
			//pCameraView2 = view2->pCameraView;

			CameraView cameraView1, cameraView2;
			cameraView1 = view1->cameraView;
			cameraView2 = view2->cameraView;

			//if (Match(pSurfel1, pSurfel2, pCameraView1, pCameraView2))	// SUNCG -v1
			if (Match(pSurfel1, pSurfel2, cameraView1, cameraView2))		//check if the current pair of the surfels are coplanar
			{
				//struct SurfelPairs SurfelPairs;
				CrspSurfels *CrspSurfels_temp = new CrspSurfels;
				
				CrspSurfels_temp->viewID_1 = viewID_1;
				CrspSurfels_temp->surfelID_vw1 = SurfelID_vw1;
				CrspSurfels_temp->viewID_2 = viewID_2;
				CrspSurfels_temp->surfelID_vw2 = SurfelID_vw2;

				crspSurfels.push_back(CrspSurfels_temp);				   //Save surfels as coresponding pair if the surfel form the first view corresponds to the same surfel in the second view

			}
			
			//printf("%d\t%d\n", n, m);
		}

		//printf("%d\n", n);
	}

}

//Testing surfels coplanarity 
bool VNLocalizer::Match(Surfel *pSurfel1, Surfel *pSurfel2, CameraView cameraView1, CameraView cameraView2)
{
	//papir BTA = 0TB^-1 * 0TA --> iz toga je prvi dio rotacijska matrica a zandji stupac je BtA
	float Rfi[9];
	float tfi[3];
	float temp[3];
	RVLCOMPTRANSF3DWITHINV(cameraView2.R, cameraView2.t, cameraView1.R, cameraView1.t, Rfi, tfi, temp);		//from the first view to second with inverse of the second view 

	float N12[3];
	RVLMULMX3X3VECT(Rfi, pSurfel1->N, N12);

	if (RVLDOTPRODUCT3(N12, pSurfel2->N) < COS45)
		return false;

	//parameters: covariances (sigma squared)
	//from Surfel 1
	float cr_sq = pow(0.01, 2);
	float c11 = cr_sq/(pow(pSurfel1->r1,2)+cr_sq);
	float c12 = cr_sq / (pow(pSurfel1->r2, 2) + cr_sq);
	
	//from Surfel 2
	float c21 = cr_sq / (pow(pSurfel2->r1, 2) + cr_sq);
	float c22 = cr_sq / (pow(pSurfel2->r2, 2) + cr_sq);

	//P matrix
	float p1 = pow(2*3.14/180,2);
	float p12 = 0.0;
	float p21 = 0.0;
	float p2 = 0.03;
	
	//variables needed for computation of Q, wehre Q is total covariance matrix
	float E2i[9];			// Used for creating identity matrix
	float E2i_t[9];
	float Cq1[9];
	float Cq2[9];
	
	float epsilon = 11.34;	//coplanarity threshold

	// E' = -I (3x3)
	RVLUNITMX3(E2i);				// I - identity matrix as array
	RVLSCALEMX3X3(E2i, -1, E2i)	
	
	// E't
	RVLCOPYMX3X3T(E2i, E2i_t);

	//Cq - diagonal covariance matrix; c1, c2, c3 = const, surfel1
	RVLDIAGMX3(c11, c12, cr_sq, Cq1);

	//Cq' - diagonal covariance matrix; c1, c2, c3 = const, surfel2
	RVLDIAGMX3(c21, c22, cr_sq, Cq2);

	//P - p1 = rotation uncertainty, p2 = pozition uncertainty; p12=p21'
	cv::Mat P(6, 6, CV_32FC1);
	P = P.zeros(6, 6, CV_32FC1);
	float *P_ = (float *)(P.data);

	P.at<float>(0, 0) = p1;
	P.at<float>(1, 1) = p1;
	P.at<float>(2, 2) = p1;
	P.at<float>(3, 3) = p2;
	P.at<float>(4, 4) = p2;
	P.at<float>(5, 5) = p2;

#ifdef RVLVNLOCALIZER_MATCH_DEBUG
	cout << "P row: 0~6 = " << endl << " " << P.rowRange(0, 6) << endl << endl;
#endif
	
	//orientation of surfel from the first view 1 (orientation of the camera RF w.r.t. surfel RF)
	//[x | y]T from the second view, 
	cv::Mat R1(3, 3, CV_32FC1);		//Rotation matrix (3x3) containing x, y, z vectors of the first view
	std::memcpy(R1.data, &pSurfel1->R, 3 * 3 * sizeof(float));
	R1 = R1.t();
	float *R1_ = (float *)(R1.data);

#ifdef RVLVNLOCALIZER_MATCH_DEBUG
	cout << "R1 row: 0~3 = " << endl << " " << R1.rowRange(0, 3) << endl << endl;
#endif
	
	cv::Mat R1xy;
	R1xy.create(3, 2, CV_32FC1);
	R1xy = R1(cv::Rect(0, 0, 2, 3));	//Copying x, y from the rotation matrix to matrix xy1
	float *R1xy_ = (float *)(R1xy.data);

	cv::Mat R1xy_t;
	R1xy_t.create(2, 3, CV_32FC1);
	R1xy_t = R1xy.t();
	float *R1xy_t_ = (float *)(R1xy_t.data);

	cv::Mat R1z;
	R1z.create(3, 1, CV_32FC1);
	R1.col(2).copyTo(R1z.col(0));
	float *R1z_ = (float *)(R1z.data);

	cv::Mat R1z_t;
	R1z_t.create(1, 3, CV_32FC1);
	R1z_t = R1z.t();
	float *R1z_t_ = (float *)(R1z_t.data);

	//orientation of surfel from the second view (orientation of the camera RF w.r.t. surfel RF)
	cv::Mat R2(3, 3, CV_32FC1);		//Rotation matrix (3x3) containing x, y, z vectors of the first view
	std::memcpy(R2.data, &pSurfel2->R, 3 * 3 * sizeof(float));
	R2 = R2.t();
	float *R2_ = (float *)(R2.data);

#ifdef RVLVNLOCALIZER_MATCH_DEBUG
	cout << "R2 row: 0~3 = " << endl << " " << R2.rowRange(0, 3) << endl << endl;
#endif

	cv::Mat R2xy;
	R2xy.create(3, 2, CV_32FC1);
	R2xy = R2(cv::Rect(0, 0, 2, 3));	//Copying x, y from the rotation matrix to matrix xy1
	float *R2xy_ = (float *)(R2xy.data);

	cv::Mat R2xy_t;
	R2xy_t.create(2, 3, CV_32FC1);
	R2xy_t = R2xy.t();
	float *R2xy_t_ = (float *)(R2xy_t.data);

	cv::Mat R2z;
	R2z.create(3, 1, CV_32FC1);
	R2.col(2).copyTo(R2z.col(0));
	float *R2z_ = (float *)(R2z.data);

	cv::Mat R2z_t;
	R2z_t.create(1, 3, CV_32FC1);
	R2z_t = R2z.t();
	float *R2z_t_ = (float *)(R2z_t.data);

	//t - translation vector (pose of the camera RF w.r.t. surfel RF)
	
	cv::Mat t1;
	t1.create(3, 1, CV_32FC1);		//translation vector matrix (3x3) containing surfel centroid coordinates (x, y, z) observed in the first view 
	std::memcpy(t1.data, &(pSurfel1->P), 3 * 1 * sizeof(float));

#ifdef RVLVNLOCALIZER_MATCH_DEBUG
	cout << "t1 row: 0~3 = " << endl << " " << t1.rowRange(0, 3) << endl << endl;
#endif

	cv::Mat t1_t;
	t1_t.create(1, 3, CV_32FC1);
	t1_t = t1.t();
	float *t1_t_ = (float *)(t1_t.data);

	cv::Mat t2;
	t2.create(3, 1, CV_32FC1);		//translation vector matrix (3x3) containing surfel centroid coordinates (x, y, z) observed in the second view 
	std::memcpy(t2.data, &(pSurfel2->P), 3 * 1 * sizeof(float));

#ifdef RVLVNLOCALIZER_MATCH_DEBUG
	cout << "t2 row: 0~3 = " << endl << " " << t2.rowRange(0, 3) << endl << endl;
#endif

	cv::Mat t2_t;
	t2_t.create(1, 3, CV_32FC1);
	t2_t = t2.t();
	float *t2_t_ = (float *)(t2_t.data);

	//R(fi) - orientation and position of the camera relative to the reference frame (view 1) --> Sf u odnosu na Sa
	//CameraFiles rotation matrix

	cv::Mat Rfi_;
	Rfi_.create(3, 3, CV_32FC1);		//Rotation matrix (3x3) containing x, y, z vectors of the first view
	std::memcpy(Rfi_.data, Rfi, 3 * 3 * sizeof(float));
	cv::Mat Rfi_t_;
	Rfi_t_.create(3, 3, CV_32FC1);		//Rotation matrix transponsed
	Rfi_t_ = Rfi_.t();
	
	cv::Mat tfi_;
	tfi_.create(3, 1, CV_32FC1);		//Rotation matrix (3x3) containing x, y, z vectors of the first view
	std::memcpy(tfi_.data, tfi, 3 * 1 * sizeof(float));

	//fi, Surfel1 --> z
	//Jacobi na to
	float J[3];
	RVLMULMX3X3VECT(Rfi, R1z_, J);

	float Jfi[9];
	RVLSKEW(J, Jfi)
	cv::Mat Jfi_;
	Jfi_.create(3, 3, CV_32FC1);		//Rotation matrix (3x3) containing x, y, z vectors of the first view
	std::memcpy(Jfi_.data, Jfi, 3 * 3 * sizeof(float));

	//E --> calculate (3x3 matrix)
	//upper part of matrix
	cv::Mat Ea;
	Ea.create(2, 3, CV_32FC1);
	Ea = R2xy_t*Rfi_;

	cv::Mat Eb;
	Eb.create(2, 2, CV_32FC1);
	Eb = Ea*R1xy;

	cv::Mat E_up;
	E_up = E_up.zeros(2, 3, CV_32FC1);
	Eb.copyTo(E_up(cv::Rect(0, 0, 2, 2)));

	//lower part of matrix
	cv::Mat t_tmp;
	t_tmp.create(1, 3, CV_32FC1);
	
	t_tmp = t2 - tfi_;
	t_tmp = t_tmp.t();
	t_tmp = t_tmp * Rfi_;
	t_tmp = t1_t - t_tmp;

	cv::Mat dwn_tmp;
	dwn_tmp.create(1, 2, CV_32FC1);
	dwn_tmp = t_tmp * R1xy;

	cv::Mat E_dwn;
	E_dwn = E_dwn.ones(1, 3, CV_32FC1);
	dwn_tmp.copyTo(E_dwn(cv::Rect(0, 0, 2, 1)));
		
	cv::Mat E;
	E.create(3, 3, CV_32FC1);
	E_up.copyTo(E(cv::Rect(0, 0, 3, 2)));
	E_dwn.copyTo(E(cv::Rect(0, 2, 3, 1)));
	float *E_ = (float *)(E.data);
	
	//Et
	cv::Mat Et;
	Et.create(3, 3, CV_32FC1);
	Et = E.t();
	float *Et_ = (float *)(Et.data);


	//C --> calculate (3x6 matrix)
	//upper part of matrix
	cv::Mat Ca;
	Ca.create(2, 3, CV_32FC1);
	Ca = R2xy_t*Jfi_;

	cv::Mat C_up;
	C_up = C_up.zeros(2, 6, CV_32FC1);
	Ca.copyTo(C_up(cv::Rect(0, 0, 3, 2)));
	float *C_up_ = (float *)(C_up.data);

	//lower part of matrix
	cv::Mat C_dwn_tmp;
	C_dwn_tmp.create(3, 1, CV_32FC1);
	C_dwn_tmp = t2 - tfi_;
	float *C_dwn_tmp_ = (float *)(C_dwn_tmp.data);

	cv::Mat C_dwn_tmp_t;
	C_dwn_tmp_t.create(1, 3, CV_32FC1);
	C_dwn_tmp_t = C_dwn_tmp.t();
	C_dwn_tmp_t = -C_dwn_tmp_t;
	C_dwn_tmp_t = C_dwn_tmp_t*Jfi_;
	float *C_dwn_tmp_t_ = (float *)(C_dwn_tmp_t.data);

	cv::Mat C_dwn_tmp2;
	C_dwn_tmp2.create(1, 3, CV_32FC1);
	C_dwn_tmp2 = R1z_t*Rfi_t_;
	float *C_dwn_tmp2_ = (float *)(C_dwn_tmp2.data);

	cv::Mat C_dwn;
	C_dwn.create(1, 6, CV_32FC1);
	C_dwn_tmp_t.copyTo(C_dwn(cv::Rect(0, 0, 3, 1)));
	C_dwn_tmp2.copyTo(C_dwn(cv::Rect(3, 0, 3, 1)));
	float *C_dwn_ = (float *)(C_dwn.data);
	
	cv::Mat C;
	C.create(3, 6, CV_32FC1);
	C_up.copyTo(C(cv::Rect(0, 0, 6, 2)));
	C_dwn.copyTo(C(cv::Rect(0, 2, 6, 1)));
	float *C_ = (float *)(C.data);
	
	//Ct --> C transponsed
	cv::Mat Ct;
	Ct.create(6, 3, CV_32FC1);
	Ct = C.t();


	//Q --> calculate (3x3 matrix)
	float Q[9];
	float Q1[9];
	float Q2[9];
	float Q3[9];
	float Q4[9];
	float Q_tmp[9];

	//Q2 = E * Cq * Et = Q1 * Et
	RVLMXMUL3X3(E_, Cq1, Q1);
	RVLMXMUL3X3(Q1, Et_, Q2);

	//Q4 = E' * Cq * E't = Q3 * E't
	RVLMXMUL3X3(E2i, Cq2, Q3);
	RVLMXMUL3X3(Q3, E2i_t, Q4);
	
	cv::Mat Q5, Q6;
	Q5.create(3, 6, CV_32FC1);
	Q6.create(3, 3, CV_32FC1);

	Q5 = C * P;
	Q6 = Q5 * Ct;
	float *Q6_ = (float *)(Q6.data);

	// Q = Q2 + Q4 + Q6
	RVLSUMMX3X3(Q2, Q4, Q_tmp);
	RVLSUMMX3X3(Q_tmp, Q6_, Q);
	cv::Mat Q_;
	Q_.create(3, 3, CV_32FC1);
	std::memcpy(Q_.data, Q, 3 * 3 * sizeof(float));


	//e --> calculate
	//upper part
	cv::Mat e_up;
	e_up.create(2, 1, CV_32FC1);
	e_up = Ea * R1z;
	float *e_up_ = (float *)(e_up.data);

	//lower part
	cv::Mat e_dwn;
	e_dwn.create(1, 1, CV_32FC1);
	e_dwn = t_tmp * R1z;
	float *e_dwn_ = (float *)(e_dwn.data);
	
	cv::Mat e;
	e.create(3, 1, CV_32FC1);
	e_up.copyTo(e(cv::Rect(0, 0, 1, 2)));
	e_dwn.copyTo(e(cv::Rect(0, 2, 1, 1)));
	float *e_ = (float *)(e.data);

	//et --> e transponsed
	cv::Mat et;
	et.create(1, 3, CV_32FC1);
	et = e.t();
	float *et_ = (float *)(et.data);

	
	//solve Q^(-1)*e, formulated as Qy=e
	cv::Mat y;
	y.create(3, 1, CV_32FC1);
	cv::solve(Q_, e, y);
	float *y_ = (float *)(y.data);
	
	//epsilon --> calculate (eT*y = epsilon)
	epsilon = RVLDOTPRODUCT3(e_, y_);
	
	// testing coplanarity constraint (eT*y <= epsilonP)
	if (epsilon <= 11.34)
		return true;	//surfels are coplanar
	else
		return false;
}

void VNLocalizer::CorrespondingSurfelPairs(View* view1, View* view2, int viewID_1, int viewID_2)
{
	
	// View1 (S1_vw1, S2_vw1) -->  View2 (S1_vw2, S2_vw2)
	for (int surfelPairs_vw1_ID = 0; surfelPairs_vw1_ID < view1->surfelPairs_list.size(); surfelPairs_vw1_ID++)		//Iterator for generated surfel pairs in the first view
	{
		int S1_vw1_ID, S2_vw1_ID;
		S1_vw1_ID = view1->surfelPairs_list[surfelPairs_vw1_ID]->S1_ID;
		S2_vw1_ID = view1->surfelPairs_list[surfelPairs_vw1_ID]->S2_ID;
		
		//Searching for the first corresponding surfel from current surfel pair (S1_vw1 --> S1_vw2)			
		for (int crspSurfel_S1_ID = 0; crspSurfel_S1_ID < crspSurfels.size(); crspSurfel_S1_ID++)
		{
			if (viewID_2 == crspSurfels[crspSurfel_S1_ID]->viewID_2 && S1_vw1_ID == crspSurfels[crspSurfel_S1_ID]->surfelID_vw1)
			{
				int S1_vw2_ID;
				S1_vw2_ID = crspSurfels[crspSurfel_S1_ID]->surfelID_vw2;

				//Searching for the second corresponding surfel from current surfel pair (S2_vw1 --> S2_vw2)	
				for (int crspSurfel_S2_ID = 0; crspSurfel_S2_ID < crspSurfels.size(); crspSurfel_S2_ID++)
				{
					if (viewID_2 == crspSurfels[crspSurfel_S2_ID]->viewID_2 && S2_vw1_ID == crspSurfels[crspSurfel_S2_ID]->surfelID_vw1)
					{
						int S2_vw2_ID;
						S2_vw2_ID = crspSurfels[crspSurfel_S2_ID]->surfelID_vw2;

						//Check if detected corresponding pair is on the Surfel Pairs list form VW2
						for (int surfelPairs_vw2_ID = 0; surfelPairs_vw2_ID < view2->surfelPairs_list.size(); surfelPairs_vw2_ID++)
						{
							int S1_vw2, S2_vw2;
							S1_vw2 = view2->surfelPairs_list[surfelPairs_vw2_ID]->S1_ID;
							S2_vw2 = view2->surfelPairs_list[surfelPairs_vw2_ID]->S2_ID;

							if ((S1_vw2 == S1_vw2_ID && S2_vw2 == S2_vw2_ID) || (S1_vw2 == S2_vw2_ID && S2_vw2 == S1_vw2_ID))
							{
								//check if corresponding surfel pair in the second view has angle >= 45�
								float CosAngle = cos(45.00 * PI / 180.0);
								float CosAngleBetweenSurfels = 0.0;


								Surfel *pS1_vw2 = &view2->surfels.NodeArray.Element[S1_vw2_ID];			//First Corresponding Surfel - Sj
								Surfel *pS2_vw2 = &view2->surfels.NodeArray.Element[S2_vw2_ID];			//Second Corresponding Surfel - Sl

								CosAngleBetweenSurfels = RVLDOTPRODUCT3(pS1_vw2->N, pS2_vw2->N);

								if (CosAngleBetweenSurfels < CosAngle)
								{
									//Surfel pair in the view1
									Surfel *pS1_vw1 = &view1->surfels.NodeArray.Element[S1_vw1_ID];		//Si
									Surfel *pS2_vw1 = &view1->surfels.NodeArray.Element[S2_vw1_ID];		//Sk

									//Check if surfel pair and corresponding surfel pair meet planar surface pair constraint
									if (PlanarSurfacePairs(pS1_vw1, pS2_vw1, pS1_vw2, pS2_vw2))
									{

										CrspSurfelPairs *CrspSurfelPairs_temp = new CrspSurfelPairs;
										//struct CrspSurfelPairs CrspSurfelPairs;

										CrspSurfelPairs_temp->viewID_1 = viewID_1;
										CrspSurfelPairs_temp->S1_ID_vw1 = S1_vw1_ID;
										CrspSurfelPairs_temp->S2_ID_vw1 = S2_vw1_ID;

										CrspSurfelPairs_temp->viewID_2 = viewID_2;
										CrspSurfelPairs_temp->S1_ID_vw2 = S1_vw2_ID;
										CrspSurfelPairs_temp->S2_ID_vw2 = S2_vw2_ID;

										//Calculating the uncertainties of the planar surface normals
										float sigma_vw1, sigma_vw2, sigma_sum;
										sigma_vw1 = PairRegValue(pS1_vw1, pS2_vw1);
										sigma_vw2 = PairRegValue(pS1_vw2, pS2_vw2);
										sigma_sum = sigma_vw1 + sigma_vw2;

										CrspSurfelPairs_temp->sigma_sum = sigma_sum;

										crspPairs.push_back(CrspSurfelPairs_temp);
										break;
									}

								}
							}
						}
					
					}

				}
			}

		}
	}


	//Finding the best pair of pairs (min(sigma_sum))
	int bestPair_ID = -1;
	float sigma_min = 1;

	for (int i = 0; i < crspPairs.size(); i++)
	{
		if (crspPairs[i]->viewID_2 == viewID_2)
		{
			if (crspPairs[i]->sigma_sum < sigma_min)
			{
				sigma_min = crspPairs[i]->sigma_sum;
				bestPair_ID = i;
			}
		}

	}

	if (bestPair_ID < 0)
		return;
	
	CrspSurfelPairs *CrspSurfelPairs_best = new CrspSurfelPairs;
	CrspSurfelPairs_best->viewID_1 = viewID_1;
	CrspSurfelPairs_best->S1_ID_vw1 = crspPairs[bestPair_ID]->S1_ID_vw1;
	CrspSurfelPairs_best->S2_ID_vw1 = crspPairs[bestPair_ID]->S2_ID_vw1;
	CrspSurfelPairs_best->viewID_2 = viewID_2;
	CrspSurfelPairs_best->S1_ID_vw2 = crspPairs[bestPair_ID]->S1_ID_vw2;
	CrspSurfelPairs_best->S2_ID_vw2 = crspPairs[bestPair_ID]->S2_ID_vw2;
	CrspSurfelPairs_best->sigma_sum = crspPairs[bestPair_ID]->sigma_sum;

	crspPairs_best.push_back(CrspSurfelPairs_best);

	//TwoPointCloudReg
	//R(fi) - orientation and position of the camera relative to the reference frame (view 1) --> Sf u odnosu na Sa
	//CameraFiles rotation matrix

	//papir BTA = 0TB^-1 * 0TA --> iz toga je prvi dio rotacijska matrica a zandji stupac je BtA
	float Rfi[9];
	float tfi[3];
	float temp[3];
	
	
	RVLCOMPTRANSF3DWITHINV(view2->cameraView.R, view2->cameraView.t, view1->cameraView.R, view1->cameraView.t, Rfi, tfi, temp);		//from the first view to second with inverse of the second view 
	cv::Mat Rfi_;
	Rfi_.create(3, 3, CV_32FC1);		//Rotation matrix (3x3) containing x, y, z vectors of the first view
	std::memcpy(Rfi_.data, Rfi, 3 * 3 * sizeof(float));
	cv::Mat Rfi_t_;
	Rfi_t_.create(3, 3, CV_32FC1);		//Rotation matrix transponsed
	Rfi_t_ = Rfi_.t();

	cv::Mat tfi_;
	tfi_.create(3, 1, CV_32FC1);		//Rotation matrix (3x3) containing x, y, z vectors of the first view
	std::memcpy(tfi_.data, tfi, 3 * 1 * sizeof(float));

	float R[9] = {};
	float t[3];
	std::memcpy(R, Rfi, 3 * 3 * sizeof(float));
	std::memcpy(t, tfi, 3 * 1 * sizeof(float));
	
	TwoPointCloudReg(view1, view2, CrspSurfelPairs_best, R, t);

	CrspPose *crspPose_temp = new CrspPose;
	
	crspPose_temp->viewID_1 = viewID_1;
	crspPose_temp->viewID_2 = viewID_2;
	RVLCOPYMX3X3(R, crspPose_temp->R);
	RVLCOPY3VECTOR(t, crspPose_temp->t);

	crspPoses.push_back(crspPose_temp);
}

//void VNLocalizer::CreatePairs(View *view, std::vector<struct SurfelPair*> &surfelPairs_vw)
//{
//	//Creating surfel pairs in the current view w.r.t. angle between two surfels
//	float CosAngle = cos(45.00 * PI / 180.0);
//
//	for (int surfelIndex = 0; surfelIndex < view->surfels.NodeArray.n - 1; surfelIndex++)
//	{
//		Surfel *pCurrentSurfel = &view->surfels.NodeArray.Element[surfelIndex];
//
//		if (pCurrentSurfel->size < minSurfelSize)
//			continue;
//
//		for (int CandidateSurfelIndex = surfelIndex + 1; CandidateSurfelIndex < view->surfels.NodeArray.n - 1; CandidateSurfelIndex++)
//		{
//			Surfel *pCandidateSurfel = &view->surfels.NodeArray.Element[CandidateSurfelIndex];
//			float CosAngleBetweenSurfels = 0.0;
//			//float DotProductN, N1_abs, N2_abs;
//
//			if (pCandidateSurfel->size < minSurfelSize)
//				continue;
//
//			CosAngleBetweenSurfels = RVLDOTPRODUCT3(pCurrentSurfel->N, pCandidateSurfel->N);
//			//N1_abs = sqrt(pow(pCurrentSurfel->N[0], 2) + pow(pCurrentSurfel->N[1], 2) + pow(pCurrentSurfel->N[2], 2));
//			//N2_abs = sqrt(pow(pCandidateSurfel->N[0], 2) + pow(pCandidateSurfel->N[1], 2) + pow(pCandidateSurfel->N[2], 2));
//
//			//AngleBetweenSurfels = acos(DotProductN / (N1_abs*N2_abs)) * 180.0 / PI;
//
//			if (CosAngleBetweenSurfels < CosAngle)
//			{
//				SurfelPair *SurfelPair_temp = new SurfelPair;
//
//				SurfelPair_temp->S1_ID = surfelIndex;
//				SurfelPair_temp->S2_ID = CandidateSurfelIndex;
//				
//				surfelPairs_vw.push_back(SurfelPair_temp);
//			}
//
//		}
//	}
//
//}

bool VNLocalizer::PlanarSurfacePairs(Surfel *pSi, Surfel *pSk, Surfel *pSj, Surfel *pSl)
{
	//View1 (Si, Sk), View2(Sj, Sl)
	
	float epsilon = 6.635;

	float sigma_si[9];
	float sigma_sk[9];
	float sigma_sj[9];
	float sigma_sl[9];

	float sigma_n_vw1;
	float sigma_n_vw2;
	
	//parameters: covariances (sigma squared)
	float cr_sq = pow(0.01, 2);

	//from surfels in the first view
	float c_i1 = cr_sq / (pow(pSi->r1, 2) + cr_sq);
	float c_i2 = cr_sq / (pow(pSi->r2, 2) + cr_sq);

	float c_k1 = cr_sq / (pow(pSk->r1, 2) + cr_sq);
	float c_k2 = cr_sq / (pow(pSk->r2, 2) + cr_sq);

	//from surfels in the second view
	float c_j1 = cr_sq / (pow(pSj->r1, 2) + cr_sq);
	float c_j2 = cr_sq / (pow(pSj->r2, 2) + cr_sq);

	float c_l1 = cr_sq / (pow(pSl->r1, 2) + cr_sq);
	float c_l2 = cr_sq / (pow(pSl->r2, 2) + cr_sq);

	//sigma_si, sigma_sk, sigma_sj, sigma_sl - diagonal covariance matrix; c1, c2, 0 = const, surfel1
	RVLDIAGMX3(c_i1, c_i2, 0.0, sigma_si);
	RVLDIAGMX3(c_k1, c_k2, 0.0, sigma_sk);

	RVLDIAGMX3(c_j1, c_j2, 0.0, sigma_sj);
	RVLDIAGMX3(c_l1, c_l2, 0.0, sigma_sl);

	//calculating (sigma_n)^2 for the first view (Si, Sk)
	float sigma_temp[3];
	float R_[9];
	float sigma_n_vw1_a;
	float sigma_n_vw1_b;

	RVLMULVECTMX3X3(pSi->N, pSk->R, sigma_temp);			//ni'*Rk
	RVLMULVECTMX3X3(sigma_temp, sigma_sk, sigma_temp);		//ans*sigma_sk
	RVLTRASPOSE3X3(pSk->R, R_);								//Rk'
	RVLMULVECTMX3X3(sigma_temp, R_, sigma_temp);			//ans*Rk'
	sigma_n_vw1_a = RVLDOTPRODUCT3(sigma_temp, pSi->N);		//ans*ni

	RVLMULVECTMX3X3(pSk->N, pSi->R, sigma_temp);			
	RVLMULVECTMX3X3(sigma_temp, sigma_si, sigma_temp);		
	RVLTRASPOSE3X3(pSi->R, R_);
	RVLMULVECTMX3X3(sigma_temp, R_, sigma_temp);			
	sigma_n_vw1_b = RVLDOTPRODUCT3(sigma_temp, pSk->N);		
	
	sigma_n_vw1 = sigma_n_vw1_a + sigma_n_vw1_b;

	//calculating (sigma_n)^2 for the second view (Sj, Sl)
	float sigma_n_vw2_a;
	float sigma_n_vw2_b;

	RVLMULVECTMX3X3(pSj->N, pSl->R, sigma_temp);			//ni'*Rk
	RVLMULVECTMX3X3(sigma_temp, sigma_sl, sigma_temp);		//ans*sigma_sk
	RVLTRASPOSE3X3(pSl->R, R_);								//Rk'
	RVLMULVECTMX3X3(sigma_temp, R_, sigma_temp);			//ans*Rk'
	sigma_n_vw2_a = RVLDOTPRODUCT3(sigma_temp, pSj->N);		//ans*ni

	RVLMULVECTMX3X3(pSl->N, pSj->R, sigma_temp);
	RVLMULVECTMX3X3(sigma_temp, sigma_sj, sigma_temp);
	RVLTRASPOSE3X3(pSj->R, R_);
	RVLMULVECTMX3X3(sigma_temp, R_, sigma_temp);
	sigma_n_vw2_b = RVLDOTPRODUCT3(sigma_temp, pSl->N);

	sigma_n_vw2 = sigma_n_vw2_a + sigma_n_vw2_b;

	//calculating epsilon
	float epsilon_num;
	float episilon_den;

	float epsilon_num_a;
	float epsilon_num_b;

	epsilon_num_a = RVLDOTPRODUCT3(pSi->N, pSk->N);
	epsilon_num_b = RVLDOTPRODUCT3(pSj->N, pSl->N);

	epsilon_num = epsilon_num_a + epsilon_num_b;
	epsilon_num = epsilon_num * epsilon_num;

	episilon_den = sigma_n_vw1 + sigma_n_vw2;

	epsilon = epsilon_num / episilon_den;

	// testing coplanarity constraint (epsilon <= 6.635)
	if (epsilon <= 6.635)
		return true;	//surfel pairs are coplanar
	else
		return false;
	
}

float VNLocalizer::PairRegValue(Surfel *pS1, Surfel *pS2)
{
	float sigma;
	float sigma_12, sigma_21; 

	sigma_12 = SigmaCalc(pS1, pS2);
	sigma_21 = SigmaCalc(pS2, pS1);
	
	if (sigma_12 < sigma_21)
	{
		sigma = sigma_12;
	}
	else
	{
		sigma = sigma_21;
	}

	return sigma;

}

float VNLocalizer::SigmaCalc(Surfel *pS1, Surfel *pS2)
{
	//return
	float sigma_error;

	float c_zf [3], c_xf [3], c_yf [3];
	float c_xf_len;

	//Creating reference frame (Sx)
	// c_zf = n1

	RVLCROSSPRODUCT3(pS1->N, pS2->N, c_xf);
	RVLNORM3(c_xf, c_xf_len);
	RVLSCALE3VECTOR2(c_xf, c_xf_len, c_xf);

	//c_yf = Cross(c_zf, c_xf)
	RVLCROSSPRODUCT3(pS1->N, c_xf, c_yf);

	//Rotation matrix Sx (RF) to Sc
	float R_xc[9];

	RVLCOPYTOCOL3(c_xf, 0, R_xc);
	RVLCOPYTOCOL3(c_yf, 1, R_xc);
	RVLCOPYTOCOL3(pS1->N, 2, R_xc);

	//Sx (RF) w.r.t. Sc (camera), R_fx = R_xc' * R1;
	float R_fx[9];
	float R1[9];						
	RVLCOPYMX3X3T(pS1->R, R1);			//pS1->R je unutar RVL popunjavana po recim, umejsto po stupcima --> zato je transponirano ovdje
	RVLMXMUL3X3T1(R_xc, R1, R_fx);

	//Sigma zx
	float R_fx_[4];
	float sigma_zx[4] = {};

	R_fx_[0] = R_fx[0];
	R_fx_[1] = R_fx[1];
	R_fx_[2] = R_fx[3];
	R_fx_[3] = R_fx[4];

	float sigma_s[4] = {};
	
	float cr_sq = pow(0.01, 2);
	float c11 = cr_sq / (pow(pS1->r1, 2) + cr_sq);
	float c12 = cr_sq / (pow(pS1->r2, 2) + cr_sq);

	sigma_s[0] = c11;
	sigma_s[3] = c12;

	RVLCOV2DTRANSF(sigma_s, R_fx_, sigma_zx);

	//Sx (RF) w.r.t. Sf'
	float R_f2x[9];
	float R_xf2[9];
	float x_xf2[3];

	float R2[9];
	RVLCOPYMX3X3T(pS2->R, R2);		//pS2->R je unutar RVL popunjavana po recim, umejsto po stupcima --> zato je transponirano ovdje
	RVLMXMUL3X3T1(R_xc, R2, R_f2x);
	RVLCOPYMX3X3T(R_f2x, R_xf2);
	RVLCOPYCOLMX3X3(R_xf2, 0, x_xf2);

	//The uncertainty of the z-axis of SX represented in SX 
	float sigma_s_[4] = {};

	float c21 = cr_sq / (pow(pS2->r1, 2) + cr_sq);
	float c22 = cr_sq / (pow(pS2->r2, 2) + cr_sq);

	sigma_s_[0] = c21;
	sigma_s_[3] = c22;

	//The Uncertainty of the x - axis represented in SX
	float sigma_x;

	sigma_x = (x_xf2[0] * sigma_s_[0] * x_xf2[0]) + (x_xf2[1] * sigma_s_[3] * x_xf2[1]);

	sigma_error = sigma_zx[0] + sigma_zx[3] + sigma_x;

	return sigma_error;

}

void VNLocalizer::TwoPointCloudReg(View *pView1, View *pView2, CrspSurfelPairs *CrspSurfelPairs_best, float *R, float *t)
{
	int S1_vw1_ID = CrspSurfelPairs_best->S1_ID_vw1;
	int S2_vw1_ID = CrspSurfelPairs_best->S2_ID_vw1;
	int S1_vw2_ID = CrspSurfelPairs_best->S1_ID_vw2;
	int S2_vw2_ID = CrspSurfelPairs_best->S2_ID_vw2;

	Surfel *pS1_vw1 = &pView1->surfels.NodeArray.Element[S1_vw1_ID];			
	Surfel *pS2_vw1 = &pView1->surfels.NodeArray.Element[S2_vw1_ID];
	Surfel *pS1_vw2 = &pView2->surfels.NodeArray.Element[S1_vw2_ID];			
	Surfel *pS2_vw2 = &pView2->surfels.NodeArray.Element[S2_vw2_ID];
	

	float R0[9];
	std::memcpy(R0, R, 3 * 3 * sizeof(float));

	float t0[3];
	std::memcpy(t0, t, 3 * 1 * sizeof(float));
	
	float R_[9];
	RVLCOPYMX3X3T(R0, R_);			//R_ = R(fi)'

	float t_[3];
	RVLINVTRANSL(R0, t0, t_);		// t_ = -R(fi)' * t(fi)



	cv::Mat A_eye;
	A_eye.create(6, 6, CV_32FC1);
	A_eye = A_eye.eye(6, 6, CV_32FC1);
	float *A_eye_ = (float *)(A_eye.data);

	
	
	
	for (int i = 0; i < 5; i++)
	{
		float E_sum = 0.0;
		float E;

		cv::Mat A_sum;
		A_sum.create(6, 6, CV_32FC1);
		A_sum = A_sum.zeros(6, 6, CV_32FC1);
		float *A_sum_ = (float *)(A_sum.data);

		cv::Mat b_sum;
		b_sum.create(6, 1, CV_32FC1);
		b_sum = b_sum.zeros(6, 1, CV_32FC1);
		float *b_sum_ = (float *)(b_sum.data);


		cv::Mat A;
		A.create(6, 6, CV_32FC1);
		A = A.zeros(6, 6, CV_32FC1);
		float *A_ = (float *)(A.data);
		
		cv::Mat b;
		b.create(6, 1, CV_32FC1);
		b = b.zeros(6, 1, CV_32FC1);
		float *b_ = (float *)(b.data);


		//First pair S1_vw1 & S1_vw2
		OptimalRotTrans(pView1, pS1_vw1, pView2, pS1_vw2, R_, t_, A_, b_, E);		//Unutar te funkcije samplerati broj tockcia kroz koji se prolazi --> provjeriti je li ok
		A_sum = A_sum + A;
		b_sum = b_sum + b;

		A = A.zeros(6, 6, CV_32FC1);

		//Second pair S2_vw1 & S2_vw2
		OptimalRotTrans(pView1, pS2_vw1, pView2, pS2_vw2, R_, t_, A_, b_, E);		//Unutar te funkcije samplerati broj tockcia kroz koji se prolazi --> provjeriti je li ok
		A_sum = A_sum + A;
		b_sum = b_sum + b;

		E_sum = E_sum + E;
		A_sum = A_sum + A_eye;


		//Ax = -b
		cv::Mat x;
		x.create(6, 1, CV_32FC1);
		cv::solve(A_sum, -b_sum, x);				
		float *x_ = (float *)(x.data);

		cv::Mat fi;
		fi.create(3, 1, CV_32FC1);
		x(cv::Rect(0, 0, 1, 3)).copyTo(fi);
		float *fi_ = (float *)(fi.data);

		cv::Mat s;
		s.create(3, 1, CV_32FC1);
		x(cv::Rect(0, 3, 1, 3)).copyTo(s);
		float *s_ = (float *)(s.data);
		
		float fi_norm[3];
		RVLCOPY3VECTOR(fi_, fi_norm);
		float theta;
		RVLNORM3(fi_norm, theta);

		float u[3];
		RVLSCALE3VECTOR2(fi_, theta, u);

		float dR[9];
		AngleAxisToRot<float>(u, theta, dR);		

		
		RVLMXMUL3X3(R_, dR, R_);				// R_ = dR * R_
		RVLSUM3VECTORS(t_, s_, t_);				// t_ = t_ + s

	}

	float v[3];
	RVLCROSSPRODUCT3(pS1_vw1->N, pS2_vw1->N, v);

	float v_len;
	RVLNORM3(v, v_len);
	RVLSCALE3VECTOR2(v, v_len, v);

	float a, t_dif[3];
	RVLDIF3VECTORS(t0, t_, t_dif);			// t0-t
	a = RVLDOTPRODUCT3(v, t_dif);			// a = v' * (t0-t)

	float t_tmp[3];
	RVLSCALE3VECTOR(v, a, t_tmp);			// [v' * (t0-t)]*v = t_tmp
	RVLSUM3VECTORS(t_, t_tmp, t_);			// t = t + t_tmp

	RVLCOPYMX3X3T(R_, R0);				// R_out = R'
	RVLINVTRANSL(R_, t_, t0);			// t_out = -R' * t;

	std::memcpy(R, R0, 3 * 3 * sizeof(float));
	std::memcpy(t, t0, 3 * 1 * sizeof(float));
}

void VNLocalizer::OptimalRotTrans(View *pView1, Surfel *pS1, View *pView2, Surfel *pS2, float *R, float *t, float *A, float *b, float &E)
{
	E = 0.0;

	float R0[9];
	std::memcpy(R0, R, 3 * 3 * sizeof(float));

	float t0[3];
	std::memcpy(t0, t, 3 * 1 * sizeof(float));

	cv::Mat A_sum;
	A_sum.create(6, 6, CV_32FC1);
	A_sum = A_sum.zeros(6, 6, CV_32FC1);
	float *A_sum_ = (float *)(A_sum.data);

	cv::Mat b_sum;
	b_sum.create(6, 1, CV_32FC1);
	b_sum = b_sum.zeros(6, 1, CV_32FC1);
	float *b_sum_ = (float *)(b_sum.data);

	//float A_first = 0;
	cv::Mat A_first;
	A_first.create(6, 6, CV_32FC1);
	A_first = A_first.zeros(6, 6, CV_32FC1);
	float *A_first_ = (float *)(A_first.data);
	
	//float A_second = 0;
	cv::Mat A_second;
	A_second.create(6, 6, CV_32FC1);
	A_second = A_second.zeros(6, 6, CV_32FC1);
	float *A_second_ = (float *)(A_second.data);

	//float b_first = 0;
	cv::Mat b_first;
	b_first.create(6, 1, CV_32FC1);
	b_first = b_first.zeros(6, 1, CV_32FC1);
	float *b_first_ = (float *)(b_first.data);

	//float b_second = 0;
	cv::Mat b_second;
	b_second.create(6, 1, CV_32FC1);
	b_second = b_second.zeros(6, 1, CV_32FC1);
	float *b_second_ = (float *)(b_second.data);

	//Surfel's points (S2)
	QList<QLIST::Index2> *pPtListSrc = &pS2->PtList;
	QLIST::Index2 *pPtIdxSrc = pPtListSrc->pFirst;		// pointer to the first element of pPtListSrc

	while (pPtIdxSrc)
	{
		int CurrentPointIndex = pPtIdxSrc->Idx;
		Point *pCurrentPoint = pView2->mesh.NodeArray.Element + CurrentPointIndex;

		float ei_a[3], ei_b[3];
		float ei;

		// R, t iz vw2 u vw1 (u idealnom slucaju Rfi i tfi)
		RVLMULMX3X3VECT(R0, pCurrentPoint->P, ei_a);	//R*p
		RVLSUM3VECTORS(ei_a, t0, ei_b);					//R*p + t
		ei = RVLDOTPRODUCT3(pS1->N, ei_b);				//n1' * (R*p + t)
		ei = ei - pS1->d;								//ei = n1' * (R*p + t) - d1

		E = E + (ei*ei);

		float ai_up_[3];
		RVLCROSSPRODUCT3(ei_a, pS1->N, ai_up_);

		cv::Mat ai_up;
		ai_up.create(3, 1, CV_32FC1);
		std::memcpy(ai_up.data, ai_up_, 3 * 1 * sizeof(float));
		
		cv::Mat ai_dwn;
		ai_dwn.create(3, 1, CV_32FC1);
		std::memcpy(ai_dwn.data, &pS1->N, 3 * 1 * sizeof(float));

		cv::Mat ai;
		ai.create(6, 1, CV_32FC1);
		ai_up.copyTo(ai(cv::Rect(0, 0, 1, 3)));
		ai_dwn.copyTo(ai(cv::Rect(0, 3, 1, 3)));	

		cv::Mat ai_t;
		ai_t.create(1, 6, CV_32FC1);
		ai_t = ai.t();

		A_first = ai*ai_t + A_first;
		b_first = ai*ei + b_first;

		
		pPtIdxSrc = pPtIdxSrc->pNext;
	}

	//Surfel's points (S1)
	pPtListSrc = &pS1->PtList;
	pPtIdxSrc = pPtListSrc->pFirst;		// pointer to the first element of pPtListSrc

	while (pPtIdxSrc)
	{
		int CurrentPointIndex = pPtIdxSrc->Idx;
		Point *pCurrentPoint = pView1->mesh.NodeArray.Element + CurrentPointIndex;

		float ej_a[3], ej_b[3];
		float ej;

		// R, t iz vw2 u vw1 (u idealnom slucaju Rfi i tfi)
		RVLMULMX3X3VECT(R0, pS2->N, ej_a);				//n2'*R' = (R*n2)'
		RVLDIF3VECTORS(pCurrentPoint->P, t0, ej_b);		//p-t
		ej = RVLDOTPRODUCT3(ej_a, ej_b) - pS2->d;

		E = E + (ej*ej);

		float aj_up_[3];
		RVLCROSSPRODUCT3(ej_b, ej_a, aj_up_)
		
		cv::Mat aj_up;
		aj_up.create(3, 1, CV_32FC1);
		std::memcpy(aj_up.data, aj_up_, 3 * 1 * sizeof(float));

		cv::Mat aj_dwn;
		aj_dwn.create(3, 1, CV_32FC1);
		std::memcpy(aj_dwn.data, ej_a, 3 * 1 * sizeof(float));

		cv::Mat aj;
		aj.create(6, 1, CV_32FC1);
		aj_up.copyTo(aj(cv::Rect(0, 0, 1, 3)));
		aj_dwn.copyTo(aj(cv::Rect(0, 3, 1, 3)));

		aj = -aj;

		cv::Mat aj_t;
		aj_t.create(1, 6, CV_32FC1);
		aj_t = aj.t();

		A_second = aj*aj_t + A_second;
		b_second = aj*ej + b_second;

		pPtIdxSrc = pPtIdxSrc->pNext;
	}

	A_sum = A_first + A_second;
	std::memcpy(A, A_sum.data, 6 * 6 * sizeof(float));

	b_sum = b_first + b_second;
	std::memcpy(b, b_sum.data, 1 * 6 * sizeof(float));

	//return A,b,E
}
#endif // not AR20
//float VNLocalizer::MultiplePointCloudReg()
//{
//
//}

//void VNLocalizer::ComputeNextView()
//{
//
//}
//
//void VNLocalizer::InformationContent()
//{
//
//}



void VNLocalizer::InformationContent(View *view)
{
	// Covariance
	float C_diagonal[3];
	float C[3] = {};

	for (int ptIndex = 0; ptIndex < view->centralPoints_array.n; ptIndex++)
	{
		//Point pCurrentPoint = &(view->centralPoints_array.Element + ptIndex);

		//// C = n*n', calculate only diagonal elememnts
		//C_diagonal[0] = pCurrentPoint[0] * &pCurrentPoint[0];
		//C_diagonal[1] = pCurrentPoint[1] * &pCurrentPoint[1];
		//C_diagonal[2] = pCurrentPoint[2] * &pCurrentPoint[2];

		//RVLSUM3VECTORS(C_diagonal, C, C);
	}



}


void VNLocalizer::UnexploredRegions(ProjectionCube *cube, float *R_cameraView, float *t_cameraView)
{
	// Projection Cube properties
	int cubeHalfSize = cube->cubeHalfSize;
	int noCubeSideRows = 2 * cubeHalfSize + 1;
	int noCubeSideColumns = 2 * cubeHalfSize + 1;

	for (int cubeSide = 0; cubeSide < 6; cubeSide++)				//for every projectionCube side
	{
		for (int cubeSideRow = 0; cubeSideRow < noCubeSideRows; cubeSideRow++)
		{
			for (int cubeSideColumn = 0; cubeSideColumn < noCubeSideColumns; cubeSideColumn++)
				//for (int cubeSideColumn = cubeSide; cubeSideColumn < (noCubeSideColumns * 6); cubeSideColumn += 6)
			{
				float cubeCell[3];
				
				// Get cooridnates of central point of current cubeCell
				cubeCell[2] = cubeHalfSize + 0.5;		// z
				cubeCell[0] = (cubeSideColumn - cubeHalfSize) * cubeCell[2] / cubeHalfSize;		// x
				cubeCell[1] = (cubeSideRow - cubeHalfSize) * cubeCell[2] / cubeHalfSize;		// y

				// Transform cubeCell central point to cube's coordinates with RCube
				float cubeCell_transformed_into_cube[3];
				RVLMULMX3X3VECT(cube->RCube[cubeSide], cubeCell, cubeCell_transformed_into_cube);		// cubeCell_:trsf = RCube * cubeCell

				// Check where does cubeCell_transformed project onto image
				float cubeCell_transformed_into_image[3];
				float tmp3x1[3];
				RVLINVTRANSF3(cubeCell_transformed_into_cube, R_cameraView, t_cameraView, cubeCell_transformed_into_image, tmp3x1);

				if (cubeCell_transformed_into_image[2] <= 0.0)
					continue;

				int u, v;
				u = (int)(round(camera.fu * cubeCell_transformed_into_image[0] / cubeCell_transformed_into_image[2] + camera.uc));	// u = fu*x/z + uc
				v = (int)(round(camera.fv * cubeCell_transformed_into_image[1] / cubeCell_transformed_into_image[2] + camera.vc));	// v = fv*y/z + vc

				if (u >= 0 && u < image_width && v >= 0 && v < image_height)
				{
					// Check in which region in the image are u and v located 
					int region = ((u > image_width / 4) && (u < 3 * image_width / 4) && (v > image_height / 4) && (v < 3 * image_height / 4) ? 2 : 1);

					int cubeRegion = cube->cubeMatrix[cubeSide][cubeSideColumn + cubeSideRow * noCubeSideColumns];

					cube->cubeMatrix[cubeSide][cubeSideColumn + cubeSideRow * noCubeSideColumns] = RVLMAX(region, cubeRegion);
				}

			}	// end cubeSideColumn

		}	// end cubeSideRow

	} // end cubeSide

}

bool VNLocalizer::CalculateScore(Array<Pair<int, int>> viewDirectionCandidates, ProjectionCube *cube, int cubeSize, float overlapInfoThrIn, float wUnexploredIn, float wOverlapIn, float *bestPanTilt, char *resultsFolder)
{
	// Parameters.
	float wUnexplored = wUnexploredIn;
	float wOverlap = wOverlapIn;
	float overlapInfoThr = overlapInfoThrIn;

	int border = 2 * (int)ceil(2.0f * camera.fu * rotErrorBound);
	//

	// Debug.

	//

	float RCamPanTilt[9] = { 0, 0, 1, -1, 0, 0, 0, -1, 0 };

	int maxNoCandidates = viewDirectionCandidates.n;
	//int cubeHalfSize = (cubeSize - 1) / 2;

	float maxScore = 0.0f;

	float PanTiltRoll[3];

	PanTiltRoll[2] = 0.0f;

	int axis[3] = { 2, 1, 0 };

	float unexplored, iUnexplored, infoContentOverlap, bestMaxInfoContentOverlap, score;
	float RPanTiltCube[9], RCamCubeNew[9];
	View *view;
	double eig[3], C[9];
	double eig_max[3], bestEig_max[3];
	bool bReal[3];
	int iEig[3], iTmp;

	overlappingViews bestOverlap;
	std::vector<int> overlapingViewsList;
	std::vector<std::vector<int>> candidates_overlapingViewsList;

	Array<int> projectedPoints_candidate;
	//projectedPoints_candidate.Element = NULL;
	projectedPoints_candidate.Element = new int[centralPointsImage.n];

	Array<int> projectedPoints_bestCandidate;
	//projectedPoints_bestCandidate.Element = NULL;
	projectedPoints_bestCandidate.Element = new int[centralPointsImage.n];


	int countThreshold = 0;

	FILE *fLogFile = fopen((std::string(resultsFolder) + "\\CalculateScore_log.txt").data(), "a");
	FILE *fProjectedPoints = fopen((std::string(resultsFolder) + "\\ProjectedPoints.txt").data(), "a");
	FILE *fCandidate;
	
	if(debug.bDebug)
		fCandidate = fopen((std::string(resultsFolder) + "\\CandidateInfo.txt").data(), "a");

	for (int iCandidate = 0; iCandidate < maxNoCandidates; iCandidate++)
	//int iCandidate = 0;
	{
		Pair<int, int> viewDirectionCandidate = viewDirectionCandidates.Element[iCandidate];
		
		// Get pan-tilt for iCandidate point
		GetPanTiltCandidate(viewDirectionCandidate, cubeSize, cube->RCube[viewDirectionCandidate.a], PanTiltRoll);
		//if (fabs(PanTiltRoll[0]) > 90 || fabs(PanTiltRoll[1]) > 90)
		//	continue;

		Rotation<float>(PanTiltRoll, axis, RPanTiltCube);
		RVLMXMUL3X3(RPanTiltCube, RCamPanTilt, RCamCubeNew);


		// UNEXPLORED REGIONS
		int nUnexplored = 0;

		// Project image of pan-tilt candidate onto the projection cube. 
		float *centralPointImage;
		int ptID;
		int iCentralPoint;
		Point centralPointsImage_Cube;
		int positionOnCube[3];

		// centralPointsImage, created in Create()

		for (ptID = 0; ptID < centralPointsImage.n; ptID++)
		{
			centralPointImage = centralPointsImage.Element[ptID].P;

			// Get unexplored regions parameters
			RVLMULMX3X3VECT(RCamCubeNew, centralPointImage, centralPointsImage_Cube.P);
			cube->FindProjectionPosition(&centralPointsImage_Cube, positionOnCube);

			// Check if that region of the projection cube is previously unexplored.
			if (cube->cubeMatrix[positionOnCube[2]][positionOnCube[0] + positionOnCube[1] * cubeSize] == 0)
				nUnexplored++;
		}

		// Calculate unexplored regions
		unexplored = (float)nUnexplored / (float)(centralPointsImage.n);


		// INFORMATION CONTENT OVERLAP
		float maxInfoContentOverlap = 0.0f;
		float covariance_sum[9];
		overlappingViews iBestOverlap;

		overlapingViewsList.clear();

		Array<int> projectedPoints;
		projectedPoints.Element = new int[centralPointsImage.n];

		// Check how much information in candidate image overlaps with all previously captured images (views)
		for (int iView = 0; iView < Views.size(); iView++)
		{
			view = Views[iView];

			RVLNULLMX3X3(covariance_sum);

			// Project image of pan-tilt candidate to the iView. 

			// Get information content overlap information
			float R_cameraView_new[9];												// from pan-tilt candidate
			RVLMXMUL3X3T1(view->cameraView.R, RCamCubeNew, R_cameraView_new);		// R_newView_to_oldView = R_oldView' * R_cam2cube

			projectedPoints.n = 0;

			if (debug.bDebug && Views.size() == debug.viewsSize && iCandidate == debug.iCandidate && iView == debug.iView)
			{
				fprintf(fCandidate, "iCandidate: %d\niView: %d", iCandidate, iView);
				fprintf(fCandidate, "\nR_panTilt2Cube:\n");
				PrintMatrix<float>(fCandidate, RCamCubeNew, 3, 3);
				fprintf(fCandidate, "\nR_iView:");
				PrintMatrix<float>(fCandidate, view->cameraView.R, 3, 3);
				fprintf(fCandidate, "\nR_iView2Cube:");
				PrintMatrix<float>(fCandidate, R_cameraView_new, 3, 3);

				fprintf(fCandidate, "\nCentral points cooridantes (iPt, x, y, z, u, v):\n");

			}

			// Go through central points of an image
			// Every image has the same central points
			for (iCentralPoint = 0; iCentralPoint < centralPointsImage.n; iCentralPoint++)
			{
				centralPointImage = centralPointsImage.Element[iCentralPoint].P;				//centralPointsImage are generted with GetCenteralPointsOfImageCells()

				Point centralPointsImage_transformed;
				RVLMULMX3X3VECT(R_cameraView_new, centralPointImage, centralPointsImage_transformed.P);	

				// Project transformed point onto image
				if (centralPointsImage_transformed.P[2] <= 0.0)
					continue;

				// Get image cell of projected image
				int image_column, image_row;
				image_column = (int)(round(camera.fu * centralPointsImage_transformed.P[0] / centralPointsImage_transformed.P[2] + camera.uc));	// u = fu*x/z + uc
				image_row = (int)(round(camera.fv * centralPointsImage_transformed.P[1] / centralPointsImage_transformed.P[2] + camera.vc));	// v = fv

				// Check if projected point is inside ROI
				if (image_column < border || image_column >= image_width - border || image_row < border || image_row >= image_height - border)
					continue;

				// Get projection cell in the cube
				int cell_column, cell_row;
				cell_column = (int)(round(camera.fu * centralPointsImage_transformed.P[0] / centralPointsImage_transformed.P[2] + camera.uc)) / cell_size_width;	// u = fu*x/z + uc
				cell_row = (int)(round(camera.fv * centralPointsImage_transformed.P[1] / centralPointsImage_transformed.P[2] + camera.vc)) / cell_size_height;		// v = fv*y/z + vc

				// spremi popis odabranih celija u array
				int iProjectedPoint = cell_row * noColumnsImage + cell_column;
				projectedPoints.Element[projectedPoints.n++] = iProjectedPoint;

				if (debug.bDebug && Views.size() == debug.viewsSize && iCandidate == debug.iCandidate && iView == debug.iView)
				{
					fprintf(fCandidate, "%d %f %f %f %d %d\n", iProjectedPoint, centralPointsImage_transformed.P[0], centralPointsImage_transformed.P[1], centralPointsImage_transformed.P[2], cell_column, cell_row);
				}

				float *covarince_cell;
				covarince_cell = view->covariance_list + (cell_column + cell_row * noColumnsImage) * 9;

				RVLSUMMX3X3UT(covarince_cell, covariance_sum, covariance_sum);

			} // end for every point

			// Calculate information content overlap
			RVLCOMPLETESIMMX3(covariance_sum);
			RVLCOPYMX3X3(covariance_sum, C);		// convert float(covariance_sum) to double

			EigCov3<double>(C, eig, bReal);
			RVLSORT3ASCEND(eig, iEig, iTmp);

			infoContentOverlap = eig[iEig[1]] / (float)(camera.w * camera.h);

			// Save overalpping views
			if (infoContentOverlap >= overlapInfoThr)
			{
				overlapingViewsList.push_back(iView);
				countThreshold++;

				if (infoContentOverlap > maxInfoContentOverlap)
				{
					maxInfoContentOverlap = infoContentOverlap;
					RVLCOPY3VECTOR(eig, eig_max);
					iBestOverlap.iView = iCandidate;
					iBestOverlap.jView = iView;

					// copy projectedPoints
					//projectedPoints_candidate.Element = NULL;
					//projectedPoints_candidate.Element = new int[projectedPoints.n];
					projectedPoints_candidate.n = projectedPoints.n;
					memcpy(projectedPoints_candidate.Element, projectedPoints.Element, sizeof(int) * projectedPoints.n);
					
					//overlappingViews_covariances.push_back(iBestOverlap); // NE!, trebam std vektor u kojeg cu za kandidata pushat s kojim viewima ima preklapanje, zato tda to kasnije mogu kopirati ako je taj kandidat odabran
				}
			}

			//if (infoContentOverlap > maxInfoContentOverlap)
			//{
			//	maxInfoContentOverlap = infoContentOverlap;
			//	RVLCOPY3VECTOR(eig, eig_max);
			//	bestOverlap.iView = iView;
			//	bestOverlap.jView = iCandidate;
			//}

			//// Save overalpping views
			//if (infoContentOverlap >= overlapInfoThr)
			//{
			//	overlapingViewList.push_back(iView);
			//	countThreshold++;
			//}
			
		}	// end for every view

		delete[] projectedPoints.Element;

		if (overlapingViewsList.size() > 0)
		{
			score = wOverlap * maxInfoContentOverlap + wUnexplored * unexplored;

			if (score > maxScore)
			{
				maxScore = score;
				bestPanTilt[0] = PanTiltRoll[0];
				bestPanTilt[1] = PanTiltRoll[1];

				overlappingViews_list = overlapingViewsList;
				RVLCOPY3VECTOR(eig_max, bestEig_max);
				bestMaxInfoContentOverlap = maxInfoContentOverlap;
				bestOverlap.iView = iBestOverlap.iView;				// iCandidate
				bestOverlap.jView = iBestOverlap.jView;				// iView
				
				// copy projectedPoints_candidate
				//projectedPoints_bestCandidate.Element = NULL;
				//projectedPoints_bestCandidate.Element = new int[projectedPoints_candidate.n];
				projectedPoints_bestCandidate.n = projectedPoints_candidate.n;

				memcpy(projectedPoints_bestCandidate.Element, projectedPoints_candidate.Element, sizeof(int) * projectedPoints_candidate.n);

				//projectedPoints_candidate.Element = NULL;
				//delete[] projectedPoints_candidate.Element;
				
				
				//bestOverlappingViews.push_back(bestOverlap);

				//view = Views[bestOverlap.iView];
				//crspPts_list = overlappingViews_list[bestOverlap.iView];

				// Copy overalpping inforamtion (point indices) of chosen iCandidate and iView to relevant view (iView)
				//for (int i = 0; i < crspPts_list.size; i++)
				//{
				//	view->crspPts_list.push_back(crspPts_list[i]);
				//}

				//view->crspPts_list(overlappingViews_list[bestOverlap.iView]);


			}
		}

		
		candidates_overlapingViewsList.push_back(overlapingViewsList);

	} // end for every candidate

	if (debug.bDebug)
	{
		fclose(fCandidate);
	}

	if (overlappingViews_list.size() == 0)
	{
		printf("No overlap\n\n");
		fprintf(fLogFile, "No overlap\n\n");
		fclose(fLogFile);

		bestPanTilt[0] = 0;
		bestPanTilt[1] = 0;

		delete[] projectedPoints_candidate.Element;
		delete[] projectedPoints_bestCandidate.Element;
		return false;
	}
	else
	{
		fprintf(fLogFile, "Unexplored: %f\nEigen values: %f %f %f\nmaxInfoContentOverlap: %f\nScore: %f\niCandidate: %d\niView: %d\nPan: %f\nTilt: %f\n\n", unexplored, bestEig_max[0], bestEig_max[1], bestEig_max[2], bestMaxInfoContentOverlap, maxScore, bestOverlap.iView, bestOverlap.jView, bestPanTilt[0], bestPanTilt[1]);

		printf("\nNo. of candidates with infoContent >= threshold: %d\n", countThreshold);
		printf("No. of overlaping views: %d\n", overlappingViews_list.size());

		fclose(fLogFile);

		for (int iPt = 0; iPt < projectedPoints_bestCandidate.n; iPt++)
		{
			fprintf(fProjectedPoints, "%d ", projectedPoints_bestCandidate.Element[iPt]);
		}

		fprintf(fProjectedPoints, "\n");
		fclose(fProjectedPoints);

		delete[] projectedPoints_candidate.Element;
		delete[] projectedPoints_bestCandidate.Element;
		return true;
	}
}

void VNLocalizer::GetPanTiltCandidate(Pair<int, int> viewDirectionCandidate, int cubeSize, float RCube[9], float *PanTilt)
{

	int cubeHalfSize = (cubeSize - 1) / 2;
	
	// Cube side
	int cubeSide = viewDirectionCandidate.a;

	// Cube cell
	int cubeCell_index = viewDirectionCandidate.b;
	int cubeSideRow = cubeCell_index / cubeSize;
	int cubeSideColumn = cubeCell_index % cubeSize;

	// Get cooridnates of central point of current cubeCell
	float cubeCell[3];

	cubeCell[2] = cubeHalfSize + 0.5;				// z
	cubeCell[0] = cubeSideColumn - cubeHalfSize;	// x
	cubeCell[1] = cubeSideRow - cubeHalfSize;		// y

	// Transform cubeCell central point to cube's coordinates with RCube'
	float cubeCell_transformed_into_cube[3];
	RVLMULMX3X3VECT(RCube, cubeCell, cubeCell_transformed_into_cube);		// cubeCell_:trsf = RCube * cubeCell

	//// Chech where does cubeCell_transformed project onto image
	//float cubeCell_transformed_into_image[3];
	//float tmp3x1[3];
	////RVLINVTRANSF3(cubeCell_transformed_into_cube, R_cameraView, t_cameraView, cubeCell_transformed_into_image, tmp3x1);
	//RVLTRANSF3(cubeCell_transformed_into_cube, R_cameraView, t_cameraView, cubeCell_transformed_into_image, tmp3x1);

	// Get new pan
	float fi = atan2(cubeCell_transformed_into_cube[1], cubeCell_transformed_into_cube[0]) * 180 / PI;	// fi = arctg(y/x) [deg]

	// Get new tilt
	// r = sqrt(x^2 + y^2 + z^2)
	float r = sqrt(RVLDOTPRODUCT3(cubeCell_transformed_into_cube, cubeCell_transformed_into_cube));
	float theta = asin(-cubeCell_transformed_into_cube[2] / r) * 180 / PI;	// theta = arccos(-z/r) [deg]

	PanTilt[0] = fi;		// pan
	PanTilt[1] = theta;		// tilt
}



void VNLocalizer::ImagesSampling()
{
	// Find overlaps between all views
	// Creates O_kl lists -> list of overlapping points between two overlaping views (kView and lView)
	// Every kView has O_k list which is std vector of O_kl list 
	ImagesOverlap();
	
	// Get total overlaping covariance matrix between overlaping  views
	OverlappingImagesCovariance();		// covarinace_sum_kl = n_kl * n_kl' for n_k2l = R_k2l * n_k;

	// Get information content factor for every overlapping point from O_kl
	InforamtionContentFactor();

	// Sample image points
	// Creates C_kl lists -> list of sampled overlapping points between two overlaping views (kView and lView)
	// Every kView has C_k list which is std vector of C_kl list 
	SamplePoints();
}

void VNLocalizer::ImagesOverlap()
{
	// Params.
	
	int border = 32;

	//
	
	int noViews = Views.size();

	// Creates list of overlapping  points (O_kl) between overlapping views with infoContentOverlap >= overlapInfoThr
	for (int kView = 0; kView < noViews; kView++)
	{
		int noOverlappingViews = Views[kView]->overlapingViewList.size();

		for (int l = 0; l < noOverlappingViews; l++)
		{
			int lView = Views[kView]->overlapingViewList[l];
						
			//overlappingPoints overlappingPt;
			View::overlappingPoints overlappingPts;
			Views[kView]->O_kl_list.clear();

			// fore every point in the k-th view
			for (int ptID_k = 0; ptID_k < Views[kView]->mesh.NodeArray.n; ptID_k++)
			{
				Point *point_k = Views[kView]->mesh.NodeArray.Element + ptID_k;

				if (ptID_k == 33)
					int debug = 1;

				if (point_k->bAmbiguous)
					continue;

				// Transform view_k to view_l
				Point point_k2l;
				float Rkl[9], tkl[3], V3Tmp[3];
				//RVLMXMUL3X3T1(view_l->cameraView.R, view_k->cameraView.R, Rkl);
#ifdef RVLVNLOCALIZER_REGISTRATION_V2
				RVLCOMPTRANSF3DWITHINV(Views[lView]->cameraView.R0, Views[lView]->cameraView.t0, Views[kView]->cameraView.R, Views[kView]->cameraView.t, Rkl, tkl, V3Tmp);
#else
				RVLCOMPTRANSF3DWITHINV(Views[lView]->cameraView.R, Views[lView]->cameraView.t, Views[kView]->cameraView.R, Views[kView]->cameraView.t, Rkl, tkl, V3Tmp);
#endif
				RVLTRANSF3(point_k->P, Rkl, tkl, point_k2l.P);
				//RVLMULMX3X3VECT(Rkl, point_k->P, point_k2l.P);
				RVLMULMX3X3VECT(Rkl, point_k->N, point_k2l.N);

				// Project transformed point onto l-th image
				if (point_k2l.P[2] <= 0.0)
					continue;

				// Get projection position of the point from k-th view onto l-th image 
				int pixelPoint_width, pixelPoint_height;
				pixelPoint_width = (int)(round(camera.fu * point_k2l.P[0] / point_k2l.P[2] + camera.uc));		// u = fu*x/z + uc
				pixelPoint_height = (int)(round(camera.fv * point_k2l.P[1] / point_k2l.P[2] + camera.vc));		// v = fv*y/z + vc

				//if (pixelPoint_width < 0 || pixelPoint_width >= image_width || pixelPoint_height < 0 || pixelPoint_height >= image_height)
				if (pixelPoint_width < border || pixelPoint_width >= image_width - border || pixelPoint_height < border || pixelPoint_height >= image_height - border)
					continue;

				// If point_k projects onto l-th view, add indices to image overlap array
				int ptID_l = pixelPoint_height * image_width + pixelPoint_width;

//				// NEW !!
				if (Views[lView]->mesh.NodeArray.Element[ptID_l].bAmbiguous)
					continue;

				overlappingPts.iView = kView;
				overlappingPts.ptID_iView = ptID_k;
				overlappingPts.jView = lView;
				overlappingPts.ptID_jView = ptID_l;

				RVLCOPY3VECTOR(point_k2l.P, overlappingPts.point_i2j.P);
				RVLCOPY3VECTOR(point_k2l.N, overlappingPts.point_i2j.N);
				overlappingPts.point_i2j.bValid = point_k->bValid;
				overlappingPts.point_i2j.bAmbiguous = point_k->bAmbiguous;

				//O_kl_list.push_back(overlappingPt);

				Views[kView]->O_kl_list.push_back(overlappingPts);
			} // enf for points in kView	

			Views[kView]->O_k_list.push_back(Views[kView]->O_kl_list);
			Views[kView]->O_kl_list.clear();
		} //end for lView
	} // end for kView
}

void VNLocalizer::ImagesOverlap_old(View *view_k, int k, View *view_l, int l)
{
	//overlappingPoints overlappingPt;
	View::overlappingPoints overlappingPts;
	view_k->O_kl_list.clear();

	// fore every point in the k-th view
	for (int ptID_k = 0; ptID_k < view_k->mesh.NodeArray.n; ptID_k++)
	{
		Point *point_k = view_k->mesh.NodeArray.Element + ptID_k;

		if (point_k->bAmbiguous)
			continue;

		// Transform view_k to view_l
		Point point_k2l;
		float Rkl[9], tkl[3], V3Tmp[3];
		//RVLMXMUL3X3T1(view_l->cameraView.R, view_k->cameraView.R, Rkl);
		RVLCOMPTRANSF3DWITHINV(view_l->cameraView.R, view_l->cameraView.t, view_k->cameraView.R, view_k->cameraView.t, Rkl, tkl, V3Tmp);
		RVLTRANSF3(point_k->P, Rkl, tkl, point_k2l.P);
		//RVLMULMX3X3VECT(Rkl, point_k->P, point_k2l.P);
		RVLMULMX3X3VECT(Rkl, point_k->N, point_k2l.N);

		// Project transformed point onto l-th image
		if (point_k2l.P[2] <= 0.0)
			continue;

		// Get projection position of the point from k-th view onto l-th image 
		int pixelPoint_width, pixelPoint_height;
		pixelPoint_width = (int)(round(camera.fu * point_k2l.P[0] / point_k2l.P[2] + camera.uc));		// u = fu*x/z + uc
		pixelPoint_height = (int)(round(camera.fv * point_k2l.P[1] / point_k2l.P[2] + camera.vc));		// v = fv*y/z + vc

		//int border = 32;

		if (pixelPoint_width < 0 || pixelPoint_width >= image_width || pixelPoint_height < 0 || pixelPoint_height >= image_height)
		//if (pixelPoint_width < border || pixelPoint_width >= image_width - border || pixelPoint_height < border || pixelPoint_height >= image_height - border)
			continue;

		// If point_k projects onto l-th view, add indices to image overlap array
		int ptID_l = pixelPoint_height * image_width + pixelPoint_width;

		overlappingPts.iView = k;
		overlappingPts.ptID_iView = ptID_k;
		overlappingPts.jView = l;
		overlappingPts.ptID_jView = ptID_l;

		RVLCOPY3VECTOR(point_k2l.P, overlappingPts.point_i2j.P);
		RVLCOPY3VECTOR(point_k2l.N, overlappingPts.point_i2j.N);
		overlappingPts.point_i2j.bValid = point_k->bValid;
		overlappingPts.point_i2j.bAmbiguous = point_k->bAmbiguous;

		//O_kl_list.push_back(overlappingPt);

		view_k->O_kl_list.push_back(overlappingPts);
	}

}

void VNLocalizer::OverlappingImagesCovariance()
{
	// Params.
	//int border = 32;

	//
	
	float covariance_point[9];
	float covariance_kl[9];

	View::overlappingViews overlappingView_kl;

	//overlappingViews overlappingView_kl;

	int noViews = Views.size();

	for (int kView = 0; kView < noViews; kView++)
	{
		int noOverlappingViews = Views[kView]->overlapingViewList.size();

		for (int l = 0; l < noOverlappingViews; l++)
		{
			int lView = Views[kView]->overlapingViewList[l];

			int noOverlappingPoints = Views[kView]->O_k_list[l].size();
			
			RVLNULLMX3X3(covariance_kl);
			for (int iPoint = 0; iPoint < noOverlappingPoints; iPoint++)
			{
				
				//std::vector<View::overlappingPoints> O_kl_list;
				//O_kl_list = Views[kView]->O_k_list[l];

				//Point currentPoint = O_kl_list[iPoint].point_i2j;
				Point currentPoint = Views[kView]->O_k_list[l].at(iPoint).point_i2j;

				if (currentPoint.bAmbiguous)
					continue;

				RVLVECTCOV3(currentPoint.N, covariance_point);
				RVLSUMMX3X3UT(covariance_point, covariance_kl, covariance_kl);

				//printf("%f\t%f\t%f\n", currentPoint.N[0], currentPoint.N[1], currentPoint.N[2]);
				//printf("%f\t%f\t%f\n", covariance_kl[0], covariance_kl[1], covariance_kl[2]);
				//printf("%f\t%f\t%f\n", covariance_kl[3], covariance_kl[4], covariance_kl[5]);
				//printf("%f\t%f\t%f\n", covariance_kl[6], covariance_kl[7], covariance_kl[8]);
			}
			
						
			//int noOverlappingPoints = O_kl_list.size();

			//RVLNULLMX3X3(covariance_kl);
			//for (int iPoint = 0; iPoint < noOverlappingPoints; iPoint++)
			//{
			//	if (O_kl_list[iPoint].iView == kView && O_kl_list[iPoint].jView == lView)
			//	{
			//		Point currentPoint = O_kl_list[iPoint].point_i2j;

			//		RVLVECTCOV3(currentPoint.N, covariance_point);
			//		RVLSUMMX3X3UT(covariance_point, covariance_kl, covariance_kl);
			//	}
			//}

			int debug = 1;
			RVLCOMPLETESIMMX3(covariance_kl);

			// overlappingView_kl.iView = kView;
			overlappingView_kl.jView = lView;
			RVLCOPYMX3X3(covariance_kl, overlappingView_kl.covariance_ij);

			double eig[3];
			bool bReal[3];
			double C[9];

			RVLCOPYMX3X3(covariance_kl, C);

			EigCov3<double>(C, eig, bReal);

			Views[kView]->overlappingViews_covariances.push_back(overlappingView_kl);
		}
	}
}

void VNLocalizer::InforamtionContentFactor()
{
	int noViews = Views.size();

	for (int kView = 0; kView < noViews; kView++)
	{
		int noOverlappingViews = Views[kView]->overlapingViewList.size();

		for (int l = 0; l < noOverlappingViews; l++)
		{
			//int lView = Views[kView]->overlapingViewList[l];

			int noOverlappingPoints = Views[kView]->O_k_list[l].size();

			//int noOverlappingPoints = O_kl_list.size();

			float c_i = 0.0f;
			
			for (int iPoint = 0; iPoint < noOverlappingPoints; iPoint++)
			{
				//std::vector<View::overlappingPoints> O_kl_list;
				//O_kl_list = Views[kView]->O_k_list[l];

				//Point currentPoint = O_kl_list[iPoint].point_i2j;

				Point currentPoint = Views[kView]->O_k_list[l].at(iPoint).point_i2j;
				if (currentPoint.bAmbiguous)
					continue;
				
				// Calculate the information content factor for every point p � O_kl
				float NTxCOV[3];
				float weight;

				RVLMULMX3X3TVECT(Views[kView]->overlappingViews_covariances[l].covariance_ij, currentPoint.N, NTxCOV);		// NTxCOV = n_kl' * coavriance_kl = covariance_kl' * n_kl
				weight = RVLDOTPRODUCT3(NTxCOV, currentPoint.N);	// weight = NTxCOV * n_kl
				weight = 1 / weight;

				c_i += weight;

				Views[kView]->O_k_list[l].at(iPoint).infoContentFactor = weight;
				Views[kView]->O_k_list[l].at(iPoint).c = c_i;
				
				
				//if (O_kl_list[iPoint].iView == kView && O_kl_list[iPoint].jView == lView)
				//{
				//	Point currentPoint = O_kl_list[iPoint].point_i2j;
				//	
				//
				//	// Calculate the information content factor for every point p � O_kl
				//	float NTxCOV[3];
				//	float weight;

				//	RVLMULMX3X3TVECT(Views[kView]->overlappingViews_covariances[lView].covariance_ij, currentPoint.N, NTxCOV);		// NTxCOV = n_kl' * coavriance_kl = covariance_kl' * n_kl
				//	weight = RVLDOTPRODUCT3(NTxCOV, currentPoint.N);	// weight = NTxCOV * n_kl
				//	weight = 1 / weight;

				//	c_i += weight;
				//	
				//	O_kl_list[iPoint].infoContentFactor = weight;
				//	O_kl_list[iPoint].c = c_i;
				//}

			}	

			Views[kView]->overlappingViews_covariances[l].c_ij = c_i;

		}
	}
}

void VNLocalizer::SamplePoints()
{
	int sampleSize = cell_size_width * cell_size_width;

	int noViews = Views.size();

	View::overlappingPoints sampledPoint;

	for (int kView = 0; kView < noViews; kView++)
	{
		int noOverlappingViews = Views[kView]->overlapingViewList.size();

		for (int l = 0; l < noOverlappingViews; l++)
		{
			//int lView = Views[kView]->overlapingViewList[l];

			int noOverlappingPoints = Views[kView]->O_k_list[l].size();

			int Nc = Views[kView]->O_k_list[l].size() / sampleSize;

			float zeta = Views[kView]->overlappingViews_covariances[l].c_ij / Nc;	// zeta = C(O_kl)/Nc

			Views[kView]->C_kl_list.clear();

			float ci = 0.0f;

			int iPoint = 0;

			for (int u = 0; u < Nc; u++)
			{
				while (zeta * u > Views[kView]->O_k_list[l].at(iPoint).c && iPoint < Views[kView]->O_k_list[l].size())
					iPoint++;

				if (iPoint >= Views[kView]->O_k_list[l].size())
					break;

				Point currentPoint = Views[kView]->O_k_list[l].at(iPoint).point_i2j;

				// Get projection position of the point from k-th view onto l-th image 
				int pixelPoint_width, pixelPoint_height;
				pixelPoint_width = (int)(round(camera.fu * currentPoint.P[0] / currentPoint.P[2] + camera.uc));			// u = fu*x/z + uc
				pixelPoint_height = (int)(round(camera.fv * currentPoint.P[1] / currentPoint.P[2] + camera.vc));		// v = fv*y/z + vc

				//int border = 32;
				//if (pixelPoint_width < border || pixelPoint_width >= image_width - border || pixelPoint_height < border || pixelPoint_height >= image_height - border)
				{
					//add mi to Ckl
					sampledPoint.iView = kView;
					sampledPoint.ptID_iView = Views[kView]->O_k_list[l].at(iPoint).ptID_iView;
					sampledPoint.jView = Views[kView]->overlapingViewList[l];
					sampledPoint.ptID_jView = Views[kView]->O_k_list[l].at(iPoint).ptID_jView;
					sampledPoint.point_i2j = Views[kView]->O_k_list[l].at(iPoint).point_i2j;

					Views[kView]->C_kl_list.push_back(sampledPoint);
				}
			}

			//for (int iPoint = 0; iPoint < noOverlappingPoints; iPoint++)
			//{

			//	float ci = Views[kView]->O_k_list[l].at(iPoint).c;

			//	for (int u = 0; u < sampleSize; u++)
			//	{
			//		int i = 0;

			//		while ((zeta * u) > ci)
			//			i++;

			//		//add mi to Ckl
			//		sampledPoint.iView = kView;
			//		sampledPoint.ptID_iView = i;
			//		sampledPoint.jView = Views[kView]->overlapingViewList[l];
			//		sampledPoint.ptID_jView = Views[kView]->O_k_list[l].at(i).ptID_jView;
			//		sampledPoint.point_i2j = Views[kView]->O_k_list[l].at(i).point_i2j;

			//		Views[kView]->C_kl_list.push_back(sampledPoint);
			//	}
			//}

			Views[kView]->C_k_list.push_back(Views[kView]->C_kl_list);
			Views[kView]->C_kl_list.clear();
		}
	}
}

void VNLocalizer::DataAssociation()
{
	int half_cell_size_width = cell_size_width / 2;
	int ROI_width = (int)ceil(2.0f * camera.fu * rotErrorBound);
		
	float CosAngleThreshold = cos(10.00 * PI / 180.0);
	float CosAngleBetweenPoints = 0.0;
	float rThreshold = 0.03;

	//float distanceBetweenPointsThreshold = 0.9*0.9; // in meters
	float distanceBetweenPointsThreshold = 0.15*0.15; // in meters
	float angleThreshold = rotErrorBound;

	Rect<int> image;
	image.minx = 0;
	image.maxx = image_width - 1;
	image.miny = 0;
	image.maxy = image_height - 1;

	Rect<int> ROI;


	// Visualizer.
	bool bVisualizeCorrespondence = false;				//change to "true" to create visualization of correspondence

	uchar SelectionColor[] = { 0, 255, 0 };

	double kViewColor[] = { 1.0, 1.0, 1.0 };
	double lViewColor[] = {0.0, 1.0, 0.0};

	// Generate correspondences between views

	int noViews = Views.size();

	float r_k2l, r_l2l;

	for (int kView = 0; kView < noViews; kView++)
	{
		int noOverlappingViews = Views[kView]->overlapingViewList.size();

		for (int l = 0; l < noOverlappingViews; l++)
		{
			//int lView = Views[kView]->overlapingViewList[l];

			int noOverlappingPoints = Views[kView]->C_k_list[l].size();
			
			int lView = Views[kView]->overlapingViewList[l];

			//for every point ptID_jView in C_kl
			for (int iPoint = 0; iPoint < noOverlappingPoints; iPoint++)
			{
				Point point_k2l = Views[kView]->C_k_list[l].at(iPoint).point_i2j;			// transforemd point from k-th to l-th view

				if (Views[kView]->C_k_list[l].at(iPoint).ptID_iView == 33)
					int deb = 1;
				
				r_k2l = sqrt(RVLDOTPRODUCT3(point_k2l.P, point_k2l.P));

				distanceBetweenPointsThreshold = 2.0f * r_k2l * angleThreshold;

				// Get point index of transformed point from the k-th view to the l-th view
				int ROI_center_u = Views[kView]->C_k_list[l].at(iPoint).ptID_jView % image_width;
				int ROI_center_v = Views[kView]->C_k_list[l].at(iPoint).ptID_jView / image_width;

				// Set ROI around selected point
				// pixelPoint (uc, vc) is in the middle of ROI
				//ROI.minx = ROI_center_u - half_cell_size_width;
				//ROI.maxx = ROI_center_u + half_cell_size_width;
				//ROI.miny = ROI_center_v - half_cell_size_width;
				//ROI.maxy = ROI_center_v + half_cell_size_width;
				
				ROI.minx = ROI_center_u - ROI_width;
				ROI.maxx = ROI_center_u + ROI_width;
				ROI.miny = ROI_center_v - ROI_width;
				ROI.maxy = ROI_center_v + ROI_width;

				// Secures that ROI is always part of the image
				CropRect<int>(ROI, image);

				// Go through every pixel inside ROI in the l-the image
				// Find pixel index with a_min
				int pixel_ID;

				float distanceBetweenPoints_min = 1000000.0;

				int pointIndex_lView_min = -1;

				int pointIndex_kView_min;

				for (int ROI_v = ROI.miny; ROI_v <= ROI.maxy; ROI_v++)
				{
					for (int ROI_u = ROI.minx; ROI_u <= ROI.maxx; ROI_u++)
					{
						pixel_ID = ROI_u + ROI_v * image_width;

						Point point_l2l = Views[lView]->mesh.NodeArray.Element[pixel_ID];			// original point in the l-th view
						
						if (point_l2l.bAmbiguous)
							continue;

						r_l2l = sqrt(RVLDOTPRODUCT3(point_l2l.P, point_l2l.P));

						if (RVLABS(r_k2l - r_l2l) <= rThreshold)
						{
							CosAngleBetweenPoints = RVLDOTPRODUCT3(point_l2l.N, point_k2l.N);

							if (CosAngleBetweenPoints > CosAngleThreshold)
							{
								float distance[3];
								float distanceBetweenPoints;
								RVLDIF3VECTORS(point_l2l.P, point_k2l.P, distance);
								distanceBetweenPoints = sqrt(RVLDOTPRODUCT3(distance, distance));

								if (distanceBetweenPoints < distanceBetweenPointsThreshold)
								{
									if (distanceBetweenPoints < distanceBetweenPoints_min)
									{
										distanceBetweenPoints_min = distanceBetweenPoints;
										pointIndex_kView_min = Views[kView]->C_k_list[l].at(iPoint).ptID_iView;
										pointIndex_lView_min = pixel_ID;
									}
								}
							}
						}
					}
				}

				if (pointIndex_lView_min >= 0)
				{
					Pair<int, int> correspondingPoints;

					correspondingPoints.a = pointIndex_kView_min;
					correspondingPoints.b = pointIndex_lView_min;

					Views[kView]->correspondences_iView_jView.push_back(correspondingPoints);
				}
			}

			Views[kView]->correspodance_list.push_back(Views[kView]->correspondences_iView_jView);
			
			// vizualizacija izmedju dva viewa

			if (bVisualizeCorrespondence)
			{
				pVisualizer->AddMesh(Views[kView]->mesh.pPolygonData, kViewColor, Views[kView]->cameraView.R, Views[kView]->cameraView.t);

				pVisualizer->AddMesh(Views[lView]->mesh.pPolygonData, lViewColor, Views[lView]->cameraView.R, Views[lView]->cameraView.t);

				VisualizeCorrespondences(pVisualizer, Views[kView], Views[lView], Views[kView]->correspondences_iView_jView);

				pVisualizer->Run();

				pVisualizer->renderer->RemoveAllViewProps();
			}

			Views[kView]->correspondences_iView_jView.clear();
		}
	}	
}



void VNLocalizer::VisualizeCorrespondences(Visualizer *pVisualizer, View *kView, View *lView, std::vector<Pair<int, int>> correspondences)//Array<Pair<int, int>> correspondences)
{

	// Create the polydata where we will store all the geometric data
	vtkSmartPointer<vtkPolyData> linesPolyData =
		vtkSmartPointer<vtkPolyData>::New();

	// Create a vtkPoints container and store the points in it
	vtkSmartPointer<vtkPoints> pts =
		vtkSmartPointer<vtkPoints>::New();

	int iLine = 0;

	float P[3];
	double P_vw1[3], P_vw2[3];
	Point *pPt_vw1, *pPt_vw2;
	float *pP_vw1, *pP_vw2;

	int noCrsp = correspondences.size();
	//int noCrsp = correspondences.n;
	//Pair<int, int> *current_correspondence;
	int ptID_vw1, ptID_vw2;

	//correspondences.n = 0;

	for (int i = 0; i < noCrsp; i++)
	{
		ptID_vw1 = correspondences[i].a;
		ptID_vw2 = correspondences[i].b;
		//current_correspondence = correspondences.Element + i;
		//ptID_vw1 = current_correspondence->a;
		//ptID_vw2 = current_correspondence->b;

		pPt_vw1 = kView->mesh.NodeArray.Element + ptID_vw1;
		RVLTRANSF3(pPt_vw1->P, kView->cameraView.R, kView->cameraView.t, P);
		//pPt_vw1 = points_vw1.Element + ptID_vw1;		//When points are in Array<Points>

		RVLCOPY3VECTOR(P, P_vw1);
		pts->InsertNextPoint(P_vw1);

		pPt_vw2 = lView->mesh.NodeArray.Element + ptID_vw2;
#ifdef RVLVNLOCALIZER_REGISTRATION_V2
		RVLTRANSF3(pPt_vw2->P, lView->cameraView.R0, lView->cameraView.t0, P);
#else
		RVLTRANSF3(pPt_vw2->P, lView->cameraView.R, lView->cameraView.t, P);
#endif
		//pPt_vw2 = points_vw2.Element + ptID_vw2;
		RVLCOPY3VECTOR(P, P_vw2);
		pts->InsertNextPoint(P_vw2);

		iLine++;
	}


	// Add the points to the polydata container
	linesPolyData->SetPoints(pts);

	// Create lines.
	vtkSmartPointer<vtkCellArray> lines =
		vtkSmartPointer<vtkCellArray>::New();

	vtkSmartPointer<vtkUnsignedCharArray> colors =
		vtkSmartPointer<vtkUnsignedCharArray>::New();

	colors->SetNumberOfComponents(3);

	unsigned char red[3] = { 255, 0, 0 };

	int nLines = iLine;

	vtkSmartPointer<vtkLine> *line = new vtkSmartPointer<vtkLine>[nLines];

	for (iLine = 0; iLine < nLines; iLine++)
	{
		line[iLine] = vtkSmartPointer<vtkLine>::New();

		line[iLine]->GetPointIds()->SetId(0, 2 * iLine);
		line[iLine]->GetPointIds()->SetId(1, 2 * iLine + 1);

		lines->InsertNextCell(line[iLine]);

		colors->InsertNextTupleValue(red);
	}

	// Add the lines to the polydata container
	linesPolyData->SetLines(lines);

	// Color the lines.
	// SetScalars() automatically associates the values in the data array passed as parameter
	// to the elements in the same indices of the cell data array on which it is called.
	// This means the first component (red) of the colors array
	// is matched with the first component of the cell array (line 0)
	// and the second component (green) of the colors array
	// is matched with the second component of the cell array (line 1)
	linesPolyData->GetCellData()->SetScalars(colors);

	// Setup the visualization pipeline
	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();

	mapper->SetInputData(linesPolyData);

	//vtkSmartPointer<vtkActor> actor =
	//	vtkSmartPointer<vtkActor>::New();
	//actor->SetMapper(mapper);
	pVisualizer->normals = vtkSmartPointer<vtkActor>::New();
	pVisualizer->normals->SetMapper(mapper);
	pVisualizer->normals->GetProperty()->SetLineWidth(2);

	pVisualizer->renderer->AddActor(pVisualizer->normals);


	//visualizer.Run();

	delete[] line;
}



// // MULTI-VIEW ICP
float VNLocalizer::MultiPointCloudReg(float* X)
{
	// Parameters.

	float alpha = 1.0f;

	//

	int noViews = Views.size();

	cv::Mat Q;
	int noRows = 6 * noViews;
	int Q_noColumns = 6 * noViews;
	Q.create(noRows, Q_noColumns, CV_32FC1);
	Q = Q.zeros(noRows, Q_noColumns, CV_32FC1);

	cv::Mat Q_;
	Q_.create(6, 6, CV_32FC1);

	cv::Mat r;
	r.create(noRows, 1, CV_32FC1);
	r = r.zeros(noRows, 1, CV_32FC1);

	cv::Mat u;
	u.create(noRows, 6, CV_32FC1);
	u = u.zeros(noRows, 6, CV_32FC1);

	cv::Mat u_;
	u_.create(6, 6, CV_32FC1);

	//initialization
	cv::Mat g_ij_sum;
	g_ij_sum.create(6, 1, CV_32FC1);
	float *g_ij_sum_ = (float *)(g_ij_sum.data);

	cv::Mat h_ij_sum;
	h_ij_sum.create(6, 1, CV_32FC1);
	float *h_ij_sum_ = (float *)(h_ij_sum.data);

	cv::Mat A_ij_sum;
	A_ij_sum.create(6, 6, CV_32FC1);
	float *A_ij_sum_ = (float *)(A_ij_sum.data);

	cv::Mat B_ij_sum;
	B_ij_sum.create(6, 6, CV_32FC1);
	float *B_ij_sum_ = (float *)(B_ij_sum.data);

	cv::Mat D_ij_sum;
	D_ij_sum.create(6, 6, CV_32FC1);
	float *D_ij_sum_ = (float *)(D_ij_sum.data);

	cv::Mat g_ij;
	g_ij.create(6, 1, CV_32FC1);
	float *g_ij_ = (float *)(g_ij.data);

	cv::Mat h_ij;
	h_ij.create(6, 1, CV_32FC1);
	float *h_ij_ = (float *)(h_ij.data);

	cv::Mat A_ij;
	A_ij.create(6, 6, CV_32FC1);
	float *A_ij_ = (float *)(A_ij.data);

	cv::Mat B_ij;
	B_ij.create(6, 6, CV_32FC1);
	float *B_ij_ = (float *)(B_ij.data);

	cv::Mat D_ij;
	D_ij.create(6, 6, CV_32FC1);
	float *D_ij_ = (float *)(D_ij.data);

	float E_ij;
	float E_ij_sum = 0;


	int goTo_column = 0;
	int goTo_row = 0;

	int nCorrespTotal = 0;

	int *nViewCorrespondences = new int[noViews];

	memset(nViewCorrespondences, 0, noViews * sizeof(int));

	int lView;

	for (int kView = 0; kView < noViews; kView++)
	{		
		int noOverlappingViews = Views[kView]->overlapingViewList.size();

		for (int l = 0; l < noOverlappingViews; l++)
		{
			lView = Views[kView]->overlapingViewList[l];

			goTo_column = 6 * lView;
			goTo_row = 6 * kView;


			//Initialize variables by setting them to zero
			g_ij_sum = g_ij_sum.zeros(6, 1, CV_32FC1);
			h_ij_sum = h_ij_sum.zeros(6, 1, CV_32FC1);
			A_ij_sum = A_ij_sum.zeros(6, 6, CV_32FC1);
			B_ij_sum = B_ij_sum.zeros(6, 6, CV_32FC1);
			D_ij_sum = D_ij_sum.zeros(6, 6, CV_32FC1);

			int noCorrespondences = Views[kView]->correspodance_list[l].size();
			nViewCorrespondences[kView] += noCorrespondences;
			nCorrespTotal += noCorrespondences;

			for (int iCrsp = 0; iCrsp < noCorrespondences; iCrsp++)
			{
				Pair<int, int> crspPts;
				crspPts = Views[kView]->correspodance_list[l].at(iCrsp);

				int ptIndex_iView = crspPts.a;
				int ptIndex_jView = crspPts.b;

				Point *p_i = &(Views[kView]->mesh.NodeArray.Element[ptIndex_iView]);
				Point *p_j = &(Views[lView]->mesh.NodeArray.Element[ptIndex_jView]);

				float d_ik = RVLDOTPRODUCT3(p_i->N, p_i->P);

				g_ij = g_ij.zeros(6, 1, CV_32FC1);
				h_ij = h_ij.zeros(6, 1, CV_32FC1);
				A_ij = A_ij.zeros(6, 6, CV_32FC1);
				B_ij = B_ij.zeros(6, 6, CV_32FC1);
				D_ij = D_ij.zeros(6, 6, CV_32FC1);

				//MultiplePtCloud_calc
				E_ij = MultiViewICP(Views[kView], p_i, Views[lView], p_j, d_ik, g_ij_, h_ij_, A_ij_, B_ij_, D_ij_);
				g_ij_sum = g_ij_sum + g_ij;
				h_ij_sum = h_ij_sum + h_ij;
				A_ij_sum = A_ij_sum + A_ij;
				B_ij_sum = B_ij_sum + B_ij;
				D_ij_sum = D_ij_sum + D_ij;

				E_ij_sum = E_ij_sum + E_ij;
			}

			// Q - matrix
			Q_ = 2 * D_ij_sum;
			Q_.copyTo(Q(cv::Rect(goTo_column, goTo_row, 6, 6)));

			// u - matrix (blocks of Q-diagonal)
			u(cv::Rect(0, kView * 6, 6, 6)) = u(cv::Rect(0, kView * 6, 6, 6)) + A_ij_sum;
			u(cv::Rect(0, lView * 6, 6, 6)) = u(cv::Rect(0, lView * 6, 6, 6)) + B_ij_sum;

			// r - matrix
			r(cv::Rect(0, kView * 6, 1, 6)) = r(cv::Rect(0, kView * 6, 1, 6)) + g_ij_sum;
			r(cv::Rect(0, lView * 6, 1, 6)) = r(cv::Rect(0, lView * 6, 1, 6)) + h_ij_sum;
		} // end for (int lView)
	} // end for (int kView)


	// Q - matrix diagonal elements
	int du = 0;
	for (int kView = 0; kView < noViews; kView++)
	{
		u(cv::Rect(0, kView * 6, 6, 6)).copyTo(Q(cv::Rect(du, du, 6, 6)));
		du = du + 6;
	}

	// Q matrix with regard to the referent view (1st view)

	float alpha_;
	int Q_wrt_rv_stride = Q_noColumns - 6;
	cv::Mat Q_wrt_rv;
	Q_wrt_rv.create(noRows - 6, Q_wrt_rv_stride, CV_32FC1);
	Q(cv::Rect(6, 6, Q_wrt_rv_stride, noRows - 6)).copyTo(Q_wrt_rv);
	float *Q_wrt_rv_ = (float *)(Q_wrt_rv.data) + 3 * Q_wrt_rv_stride + 3;
	for (int kView = 0; kView < noViews - 1; kView++, Q_wrt_rv_ += (6 * (Q_wrt_rv_stride + 1)))
	{
		alpha_ = alpha * (float)(nViewCorrespondences[kView]);

		Q_wrt_rv_[0] += alpha_;
		Q_wrt_rv_[Q_wrt_rv_stride + 1] += alpha_;
		Q_wrt_rv_[2 * (Q_wrt_rv_stride + 1)] += alpha_;
	}

	cv::Mat r_wrt_rv;
	r_wrt_rv.create(noRows - 6, 1, CV_32FC1);
	r(cv::Rect(0, 6, 1, noRows - 6)).copyTo(r_wrt_rv);

	cv::Mat x_wrt_rv;
	x_wrt_rv.create(noRows - 6, 1, CV_32FC1);
	cv::solve(Q_wrt_rv, -r_wrt_rv, x_wrt_rv);
	float *x_wrt_rv_ = (float *)(x_wrt_rv.data);
	//RVLSCALE3VECTOR(x_wrt_rv_, 0.1, x_wrt_rv_);
	//float *x_wrt_rv_t_ = x_wrt_rv_ + 3;
	//RVLSCALE3VECTOR(x_wrt_rv_t_, 0.1, x_wrt_rv_t_);

	// solve Q^(-1)*(-r), formulated as Qx = -r wrt 1st view
	cv::Mat x;
	x.create(noRows, 1, CV_32FC1);
	x = x.zeros(noRows, 1, CV_32FC1);
	x_wrt_rv.copyTo(x(cv::Rect(0, 6, 1, noRows - 6)));
	float *x_ = (float *)(x.data);


	std::memcpy(X, x_, noViews * 6 * 1 * sizeof(float));

	printf("Cost = \t%.4f\n", E_ij_sum);

	printf("Avg. cost = \t%f\n", sqrt(E_ij_sum / (float)nCorrespTotal));

	cv::Mat expectedCostChange;
	expectedCostChange = 2 * r_wrt_rv.t() * x_wrt_rv + x_wrt_rv.t() * Q_wrt_rv * x_wrt_rv;	// expectedCostChange = 2 * r_'*x_views_+x_views_'*Q_*x_views_;
	float* expectedCostChange_ = (float *)(expectedCostChange.data);

	printf("Expected Cost Change = \t%f\n", *expectedCostChange_);

	delete[] nViewCorrespondences;

	return E_ij_sum;
}

// // MULTI-VIEW ICP
double VNLocalizer::MultiPointCloudReg(double* X)
{
	// Parameters.

	double alpha = 1.0f;

	//

	int noViews = Views.size();

	cv::Mat Q;
	int noRows = 6 * noViews;
	int Q_noColumns = 6 * noViews;
	Q.create(noRows, Q_noColumns, CV_64FC1);
	Q = Q.zeros(noRows, Q_noColumns, CV_64FC1);

	cv::Mat Q_;
	Q_.create(6, 6, CV_64FC1);

	cv::Mat r;
	r.create(noRows, 1, CV_64FC1);
	r = r.zeros(noRows, 1, CV_64FC1);

	cv::Mat u;
	u.create(noRows, 6, CV_64FC1);
	u = u.zeros(noRows, 6, CV_64FC1);

	cv::Mat u_;
	u_.create(6, 6, CV_64FC1);

	//initialization
	cv::Mat g_ij_sum;
	g_ij_sum.create(6, 1, CV_64FC1);
	double *g_ij_sum_ = (double *)(g_ij_sum.data);

	cv::Mat h_ij_sum;
	h_ij_sum.create(6, 1, CV_64FC1);
	double *h_ij_sum_ = (double *)(h_ij_sum.data);

	cv::Mat A_ij_sum;
	A_ij_sum.create(6, 6, CV_64FC1);
	double *A_ij_sum_ = (double *)(A_ij_sum.data);

	cv::Mat B_ij_sum;
	B_ij_sum.create(6, 6, CV_64FC1);
	double *B_ij_sum_ = (double *)(B_ij_sum.data);

	cv::Mat D_ij_sum;
	D_ij_sum.create(6, 6, CV_64FC1);
	double *D_ij_sum_ = (double *)(D_ij_sum.data);

	cv::Mat g_ij;
	g_ij.create(6, 1, CV_64FC1);
	double *g_ij_ = (double *)(g_ij.data);

	cv::Mat h_ij;
	h_ij.create(6, 1, CV_64FC1);
	double *h_ij_ = (double *)(h_ij.data);

	cv::Mat A_ij;
	A_ij.create(6, 6, CV_64FC1);
	double *A_ij_ = (double *)(A_ij.data);

	cv::Mat B_ij;
	B_ij.create(6, 6, CV_64FC1);
	double *B_ij_ = (double *)(B_ij.data);

	cv::Mat D_ij;
	D_ij.create(6, 6, CV_64FC1);
	double *D_ij_ = (double *)(D_ij.data);

	double E_ij;
	double E_ij_sum = 0;


	int goTo_column = 0;
	int goTo_row = 0;

	int nCorrespTotal = 0;

	int *nViewCorrespondences = new int[noViews];

	memset(nViewCorrespondences, 0, noViews * sizeof(int));

	bool *bAssociated = new bool[noViews];

	memset(bAssociated, 0, noViews * sizeof(bool));

	int lView;

	for (int kView = 0; kView < noViews; kView++)
	{
		int noOverlappingViews = Views[kView]->overlapingViewList.size();

		for (int l = 0; l < noOverlappingViews; l++)
		{
			lView = Views[kView]->overlapingViewList[l];

			goTo_column = 6 * lView;
			goTo_row = 6 * kView;


			//Initialize variables by setting them to zero
			g_ij_sum = g_ij_sum.zeros(6, 1, CV_64FC1);
			h_ij_sum = h_ij_sum.zeros(6, 1, CV_64FC1);
			A_ij_sum = A_ij_sum.zeros(6, 6, CV_64FC1);
			B_ij_sum = B_ij_sum.zeros(6, 6, CV_64FC1);
			D_ij_sum = D_ij_sum.zeros(6, 6, CV_64FC1);

			int noCorrespondences = Views[kView]->correspodance_list[l].size();
			nViewCorrespondences[kView] += noCorrespondences;
			nCorrespTotal += noCorrespondences;

			for (int iCrsp = 0; iCrsp < noCorrespondences; iCrsp++)
			{
				Pair<int, int> crspPts;
				crspPts = Views[kView]->correspodance_list[l].at(iCrsp);

				int ptIndex_iView = crspPts.a;
				int ptIndex_jView = crspPts.b;

				Point *p_i = &(Views[kView]->mesh.NodeArray.Element[ptIndex_iView]);
				Point *p_j = &(Views[lView]->mesh.NodeArray.Element[ptIndex_jView]);

				double d_ik = RVLDOTPRODUCT3(p_i->N, p_i->P);

				g_ij = g_ij.zeros(6, 1, CV_64FC1);
				h_ij = h_ij.zeros(6, 1, CV_64FC1);
				A_ij = A_ij.zeros(6, 6, CV_64FC1);
				B_ij = B_ij.zeros(6, 6, CV_64FC1);
				D_ij = D_ij.zeros(6, 6, CV_64FC1);

				//MultiplePtCloud_calc
				//E_ij = MultiViewICP(Views[kView], p_i, Views[lView], p_j, d_ik, g_ij_, h_ij_, A_ij_, B_ij_, D_ij_);
				E_ij = MultiViewICP2(Views[kView], p_i, Views[lView], p_j, d_ik, g_ij_, h_ij_, A_ij_, B_ij_, D_ij_);
				g_ij_sum = g_ij_sum + g_ij;
				h_ij_sum = h_ij_sum + h_ij;
				A_ij_sum = A_ij_sum + A_ij;
				B_ij_sum = B_ij_sum + B_ij;
				D_ij_sum = D_ij_sum + D_ij;
			}

			// Q - matrix
			//Q_ = 2 * D_ij_sum;
			Q_ = Q_.zeros(6, 6, CV_64FC1);
			Q_.copyTo(Q(cv::Rect(goTo_column, goTo_row, 6, 6)));

			//// u - matrix (blocks of Q-diagonal)
			//u(cv::Rect(0, kView * 6, 6, 6)) = u(cv::Rect(0, kView * 6, 6, 6)) + A_ij_sum;
			//u(cv::Rect(0, lView * 6, 6, 6)) = u(cv::Rect(0, lView * 6, 6, 6)) + B_ij_sum;

			//// r - matrix
			//r(cv::Rect(0, kView * 6, 1, 6)) = r(cv::Rect(0, kView * 6, 1, 6)) + g_ij_sum;
			//r(cv::Rect(0, lView * 6, 1, 6)) = r(cv::Rect(0, lView * 6, 1, 6)) + h_ij_sum;

			if (lView < kView && !bAssociated[kView])
			{
				// u - matrix (blocks of Q-diagonal)
				u(cv::Rect(0, kView * 6, 6, 6)) = u(cv::Rect(0, kView * 6, 6, 6)) + A_ij_sum;

				// r - matrix
				r(cv::Rect(0, kView * 6, 1, 6)) = r(cv::Rect(0, kView * 6, 1, 6)) + g_ij_sum;

				bAssociated[kView] = true;

				E_ij_sum = E_ij_sum + E_ij;
			}
		} // end for (int lView)
	} // end for (int kView)


	// Q - matrix diagonal elements
	int du = 0;
	for (int kView = 0; kView < noViews; kView++)
	{
		u(cv::Rect(0, kView * 6, 6, 6)).copyTo(Q(cv::Rect(du, du, 6, 6)));
		du = du + 6;
	}

	// Q matrix with regard to the referent view (1st view)

	double alpha_;
	int Q_wrt_rv_stride = Q_noColumns - 6;
	cv::Mat Q_wrt_rv;
	Q_wrt_rv.create(noRows - 6, Q_wrt_rv_stride, CV_64FC1);
	Q(cv::Rect(6, 6, Q_wrt_rv_stride, noRows - 6)).copyTo(Q_wrt_rv);
	double *Q_wrt_rv_ = (double *)(Q_wrt_rv.data) + 3 * Q_wrt_rv_stride + 3;
	for (int kView = 0; kView < noViews - 1; kView++, Q_wrt_rv_ += (6 * (Q_wrt_rv_stride + 1)))
	{
		alpha_ = alpha * (double)(nViewCorrespondences[kView]);

		Q_wrt_rv_[0] += alpha_;
		Q_wrt_rv_[Q_wrt_rv_stride + 1] += alpha_;
		Q_wrt_rv_[2 * (Q_wrt_rv_stride + 1)] += alpha_;
	}

	cv::Mat r_wrt_rv;
	r_wrt_rv.create(noRows - 6, 1, CV_64FC1);
	r(cv::Rect(0, 6, 1, noRows - 6)).copyTo(r_wrt_rv);

	cv::Mat x_wrt_rv;
	x_wrt_rv.create(noRows - 6, 1, CV_64FC1);

#ifdef RVLVNLOCALIZER_REGISTRATION_V2
	cv::Mat Q_singleBlock(6, 6, CV_64FC1);
	double *Q_singleBlock_ = (double *)(Q_singleBlock.data);
	cv::Mat r_singleBlock(6, 1, CV_64FC1);
	cv::Mat x_singleBlock(6, 1, CV_64FC1);
	for (int kView = 1; kView < noViews; kView++, Q_wrt_rv_ += (6 * (Q_wrt_rv_stride + 1)))
	{
		alpha_ = alpha * (double)(nViewCorrespondences[kView]);

		Q(cv::Rect(6 * kView, 6 * kView, 6, 6)).copyTo(Q_singleBlock);
		Q_singleBlock_[3 + 3 * 6] += alpha_;
		Q_singleBlock_[4 + 4 * 6] += alpha_;
		Q_singleBlock_[5 + 5 * 6] += alpha_;
		r(cv::Rect(0, 6 * kView, 1, 6)).copyTo(r_singleBlock);
		cv::solve(Q_singleBlock, -r_singleBlock, x_singleBlock);
		x_singleBlock.copyTo(x_wrt_rv(cv::Rect(0, 6 * (kView - 1), 1, 6)));
	}
#else
	cv::solve(Q_wrt_rv, -r_wrt_rv, x_wrt_rv);
#endif
	double *x_wrt_rv_ = (double *)(x_wrt_rv.data);
	//RVLSCALE3VECTOR(x_wrt_rv_, 0.1, x_wrt_rv_);
	//double *x_wrt_rv_t_ = x_wrt_rv_ + 3;
	//RVLSCALE3VECTOR(x_wrt_rv_t_, 0.1, x_wrt_rv_t_);

	// solve Q^(-1)*(-r), formulated as Qx = -r wrt 1st view
	cv::Mat x;
	x.create(noRows, 1, CV_64FC1);
	x = x.zeros(noRows, 1, CV_64FC1);
	x_wrt_rv.copyTo(x(cv::Rect(0, 6, 1, noRows - 6)));
	double *x_ = (double *)(x.data);


	//double alpha_;
	//int Q_wrt_rv_stride = Q_noColumns - 6;
	//cv::Mat Q_wrt_rv;
	//Q_wrt_rv.create(noRows - 6, Q_wrt_rv_stride, CV_64FC1);
	//Q(cv::Rect(0, 0, Q_wrt_rv_stride, noRows - 6)).copyTo(Q_wrt_rv);
	//double *Q_wrt_rv_ = (double *)(Q_wrt_rv.data) + 3 * Q_wrt_rv_stride + 3;
	//for (int kView = 0; kView < noViews - 1; kView++, Q_wrt_rv_ += (6 * (Q_wrt_rv_stride + 1)))
	//{
	//	alpha_ = alpha * (double)(nViewCorrespondences[kView]);

	//	Q_wrt_rv_[0] += alpha_;
	//	Q_wrt_rv_[Q_wrt_rv_stride + 1] += alpha_;
	//	Q_wrt_rv_[2 * (Q_wrt_rv_stride + 1)] += alpha_;
	//}

	//cv::Mat r_wrt_rv;
	//r_wrt_rv.create(noRows - 6, 1, CV_64FC1);
	//r(cv::Rect(0, 0, 1, noRows - 6)).copyTo(r_wrt_rv);

	//cv::Mat x_wrt_rv;
	//x_wrt_rv.create(noRows - 6, 1, CV_64FC1);
	//cv::solve(Q_wrt_rv, -r_wrt_rv, x_wrt_rv);
	//double *x_wrt_rv_ = (double *)(x_wrt_rv.data);
	////RVLSCALE3VECTOR(x_wrt_rv_, 0.1, x_wrt_rv_);
	////double *x_wrt_rv_t_ = x_wrt_rv_ + 3;
	////RVLSCALE3VECTOR(x_wrt_rv_t_, 0.1, x_wrt_rv_t_);

	//// solve Q^(-1)*(-r), formulated as Qx = -r wrt 1st view
	//cv::Mat x;
	//x.create(noRows, 1, CV_64FC1);
	//x = x.zeros(noRows, 1, CV_64FC1);
	//x_wrt_rv.copyTo(x(cv::Rect(0, 0, 1, noRows - 6)));
	//double *x_ = (double *)(x.data);



	std::memcpy(X, x_, noViews * 6 * 1 * sizeof(double));

	printf("Cost = \t%.4f\n", E_ij_sum);

	printf("Avg. cost = \t%f\n", sqrt(E_ij_sum / (double)nCorrespTotal));

	cv::Mat expectedCostChange;
	expectedCostChange = 2 * r_wrt_rv.t() * x_wrt_rv + x_wrt_rv.t() * Q_wrt_rv * x_wrt_rv;	// expectedCostChange = 2 * r_'*x_views_+x_views_'*Q_*x_views_;
	double* expectedCostChange_ = (double *)(expectedCostChange.data);

	printf("Expected Cost Change = \t%f\n", *expectedCostChange_);

	delete[] nViewCorrespondences;
	delete[] bAssociated;

	return E_ij_sum;
}



float VNLocalizer::MultiViewICP(View* iView, Point *p_i, View* jView, Point *p_j, float d_ik, float* g_ij_, float* h_ij_, float* A_ij_, float* B_ij_, float* D_ij_)
{

	float t_[3];
	float nR[3];
	float d, E_ij_;


	//T_ij = Rj*p_j + tj - ti;
	float T_ij[3];
	float Rj_pj[3];

	// Transform view_i/k to view_j/l
	//float Rij[9];
	//RVLMXMUL3X3T1(jView->cameraView.R, iView->cameraView.R, Rij);

	//RVLMULMX3X3VECT(Rij, p_j->P, Rj_pj);					// Rj_pj = Rj * pj
	RVLMULMX3X3VECT(jView->cameraView.R, p_j->P, Rj_pj);					// Rj_pj = Rj * pj
	RVLSUM3VECTORS(Rj_pj, jView->cameraView.t, t_);			// t_ = Rj_pj + tj
	RVLDIF3VECTORS(t_, iView->cameraView.t, T_ij);			// T_ij = t_ - ti

	//e_ijk = (Ri*n_ik)' * T_ij - d_ik;
	float Ri_ni[3];
	float e_ijk;
	RVLMULMX3X3VECT(iView->cameraView.R, p_i->N, Ri_ni);	// Ri_ni = (Ri*n_ik)
	e_ijk = RVLDOTPRODUCT3(Ri_ni, T_ij);					// e_ijk = Rn' * T_ij
	e_ijk = e_ijk - d_ik;

	// cost calcualtion
	E_ij_ = e_ijk * e_ijk;

	// a - matrix
	float a_ijk_up[3];
	RVLCROSSPRODUCT3(Ri_ni, T_ij, a_ijk_up);				// a_ijk_up = cross((Ri*n_ik), T_ij);

	float a_ijk_dwn[3];
	RVLNEGVECT3(Ri_ni, a_ijk_dwn);							// a_ijk_dwn = -(Ri*n_ik) = -Rn;


	cv::Mat a_up;
	a_up.create(3, 1, CV_32FC1);
	std::memcpy(a_up.data, a_ijk_up, 3 * 1 * sizeof(float));

	cv::Mat a_dwn;
	a_dwn.create(3, 1, CV_32FC1);
	std::memcpy(a_dwn.data, a_ijk_dwn, 3 * 1 * sizeof(float));

	cv::Mat a;												//a_ijk = vertcat(a_ijk_up, a_ijk_dwn);
	a.create(6, 1, CV_32FC1);
	a_up.copyTo(a(cv::Rect(0, 0, 1, 3)));
	a_dwn.copyTo(a(cv::Rect(0, 3, 1, 3)));

	// b - matrix
	float b_ijk_up[3];
	RVLCROSSPRODUCT3(Rj_pj, Ri_ni, b_ijk_up);				// b_ijk_up = cross((Rj*p_j), (Ri*n_ik));

	cv::Mat b_up;
	b_up.create(3, 1, CV_32FC1);
	std::memcpy(b_up.data, b_ijk_up, 3 * 1 * sizeof(float));

	cv::Mat b_dwn;
	b_dwn.create(3, 1, CV_32FC1);
	std::memcpy(b_dwn.data, Ri_ni, 3 * 1 * sizeof(float));

	cv::Mat b;												//b_ijk = vertcat(b_ijk_up, b_ijk_dwn);
	b.create(6, 1, CV_32FC1);
	b_up.copyTo(b(cv::Rect(0, 0, 1, 3)));
	b_dwn.copyTo(b(cv::Rect(0, 3, 1, 3)));

	// g - matrix
	cv::Mat g;
	g.create(6, 1, CV_32FC1);
	g = a * e_ijk;
	float *g_ = (float *)(g.data);

	// h - matrix
	cv::Mat h;
	h.create(6, 1, CV_32FC1);
	h = b * e_ijk;
	float *h_ = (float *)(h.data);

	// A - matrix
	cv::Mat A;
	A.create(6, 6, CV_32FC1);
	A = a * a.t();				// A = a*a';
	float *A_ = (float *)(A.data);

	// B - matrix
	cv::Mat B;
	B.create(6, 6, CV_32FC1);
	B = b * b.t();
	float *B_ = (float *)(B.data);

	// D - matrix
	cv::Mat D;
	D.create(6, 6, CV_32FC1);
	D = a * b.t();
	float *D_ = (float *)(D.data);


	float ni_Ri[3];
	RVLMULMX3X3VECT(iView->cameraView.R, p_i->N, ni_Ri);		// ni_Ri = ni'*Ri'
	d = RVLDOTPRODUCT3(ni_Ri, T_ij);

	//x_nl = d - d_ik;

	std::memcpy(g_ij_, g_, 6 * 1 * sizeof(float));
	std::memcpy(h_ij_, h_, 6 * 1 * sizeof(float));
	std::memcpy(A_ij_, A_, 6 * 6 * sizeof(float));
	std::memcpy(B_ij_, B_, 6 * 6 * sizeof(float));
	std::memcpy(D_ij_, D_, 6 * 6 * sizeof(float));


	return E_ij_;

}


double VNLocalizer::MultiViewICP(View* iView, Point *p_i, View* jView, Point *p_j, double d_ik, double* g_ij_, double* h_ij_, double* A_ij_, double* B_ij_, double* D_ij_)
{

	double t_[3];
	double nR[3];
	double d, E_ij_;


	//T_ij = Rj*p_j + tj - ti;
	double T_ij[3];
	double Rj_pj[3];

	// Transform view_i/k to view_j/l
	//double Rij[9];
	//RVLMXMUL3X3T1(jView->cameraView.R, iView->cameraView.R, Rij);

	//RVLMULMX3X3VECT(Rij, p_j->P, Rj_pj);					// Rj_pj = Rj * pj
	RVLMULMX3X3VECT(jView->cameraView.R, p_j->P, Rj_pj);	// Rj_pj = Rj * pj
	RVLSUM3VECTORS(Rj_pj, jView->cameraView.t, t_);			// t_ = Rj_pj + tj
	RVLDIF3VECTORS(t_, iView->cameraView.t, T_ij);			// T_ij = t_ - ti

	//e_ijk = (Ri*n_ik)' * T_ij - d_ik;
	double Ri_ni[3];
	double e_ijk;
	RVLMULMX3X3VECT(iView->cameraView.R, p_i->N, Ri_ni);	// Ri_ni = (Ri*n_ik)
	e_ijk = RVLDOTPRODUCT3(Ri_ni, T_ij);					// e_ijk = Rn' * T_ij
	e_ijk = e_ijk - d_ik;

	// cost calcualtion
	E_ij_ = e_ijk * e_ijk;

	// a - matrix
	double a_ijk_up[3];
	RVLCROSSPRODUCT3(Ri_ni, T_ij, a_ijk_up);				// a_ijk_up = cross((Ri*n_ik), T_ij);

	double a_ijk_dwn[3];
	RVLNEGVECT3(Ri_ni, a_ijk_dwn);							// a_ijk_dwn = -(Ri*n_ik) = -Rn;


	cv::Mat a_up;
	a_up.create(3, 1, CV_64FC1);
	std::memcpy(a_up.data, a_ijk_up, 3 * 1 * sizeof(double));

	cv::Mat a_dwn;
	a_dwn.create(3, 1, CV_64FC1);
	std::memcpy(a_dwn.data, a_ijk_dwn, 3 * 1 * sizeof(double));

	cv::Mat a;												//a_ijk = vertcat(a_ijk_up, a_ijk_dwn);
	a.create(6, 1, CV_64FC1);
	a_up.copyTo(a(cv::Rect(0, 0, 1, 3)));
	a_dwn.copyTo(a(cv::Rect(0, 3, 1, 3)));

	// b - matrix
	double b_ijk_up[3];
	RVLCROSSPRODUCT3(Rj_pj, Ri_ni, b_ijk_up);				// b_ijk_up = cross((Rj*p_j), (Ri*n_ik));

	cv::Mat b_up;
	b_up.create(3, 1, CV_64FC1);
	std::memcpy(b_up.data, b_ijk_up, 3 * 1 * sizeof(double));

	cv::Mat b_dwn;
	b_dwn.create(3, 1, CV_64FC1);
	std::memcpy(b_dwn.data, Ri_ni, 3 * 1 * sizeof(double));

	cv::Mat b;												//b_ijk = vertcat(b_ijk_up, b_ijk_dwn);
	b.create(6, 1, CV_64FC1);
	b_up.copyTo(b(cv::Rect(0, 0, 1, 3)));
	b_dwn.copyTo(b(cv::Rect(0, 3, 1, 3)));

	// g - matrix
	cv::Mat g;
	g.create(6, 1, CV_64FC1);
	g = a * e_ijk;
	double *g_ = (double *)(g.data);

	// h - matrix
	cv::Mat h;
	h.create(6, 1, CV_64FC1);
	h = b * e_ijk;
	double *h_ = (double *)(h.data);

	// A - matrix
	cv::Mat A;
	A.create(6, 6, CV_64FC1);
	A = a * a.t();				// A = a*a';
	double *A_ = (double *)(A.data);

	// B - matrix
	cv::Mat B;
	B.create(6, 6, CV_64FC1);
	B = b * b.t();
	double *B_ = (double *)(B.data);

	// D - matrix
	cv::Mat D;
	D.create(6, 6, CV_64FC1);
	D = a * b.t();
	double *D_ = (double *)(D.data);


	double ni_Ri[3];
	RVLMULMX3X3VECT(iView->cameraView.R, p_i->N, ni_Ri);		// ni_Ri = ni'*Ri'
	d = RVLDOTPRODUCT3(ni_Ri, T_ij);

	//x_nl = d - d_ik;

	std::memcpy(g_ij_, g_, 6 * 1 * sizeof(double));
	std::memcpy(h_ij_, h_, 6 * 1 * sizeof(double));
	std::memcpy(A_ij_, A_, 6 * 6 * sizeof(double));
	std::memcpy(B_ij_, B_, 6 * 6 * sizeof(double));
	std::memcpy(D_ij_, D_, 6 * 6 * sizeof(double));


	return E_ij_;

}

double VNLocalizer::MultiViewICP2(View* iView, Point *p_i, View* jView, Point *p_j, double d_ik, double* g_ij_, double* h_ij_, double* A_ij_, double* B_ij_, double* D_ij_)
{

	double t_[3];
	double nR[3];
	double d, E_ij_;


	//T_ij = Rj*p_j + tj - ti;
	double T_ij[3];
	double Ri_pi[3], Rj_pj[3];

	// Transform view_i/k to view_j/l
	//double Rij[9];
	//RVLMXMUL3X3T1(jView->cameraView.R, iView->cameraView.R, Rij);

	//RVLMULMX3X3VECT(Rij, p_j->P, Rj_pj);									// Rj_pj = Rj * pj
	RVLMULMX3X3VECT(jView->cameraView.R, p_j->P, Rj_pj);					// Rj_pj = Rj * pj
	//RVLSUM3VECTORS(Rj_pj, jView->cameraView.t, t_);						// t_ = Rj_pj + tj
	RVLDIF3VECTORS(jView->cameraView.t, iView->cameraView.t, T_ij);			// T_ij = tj - ti

	RVLMULMX3X3VECT(iView->cameraView.R, p_i->P, Ri_pi);					// Ri_pi = Ri * pi

	//e_ijk = (Ri*n_ik)' * T_ij - d_ik;
	double Ri_ni[3];
	RVLMULMX3X3VECT(iView->cameraView.R, p_i->N, Ri_ni);	// Ri_ni = (Ri*n_ik)
	double Rj_nj[3];
	RVLMULMX3X3VECT(jView->cameraView.R, p_j->N, Rj_nj);	// Rj_nj = (Rj*n_jk)
	double N[3];
	RVLSUM3VECTORS(Ri_ni, Rj_nj, N);
	float fTmp;
	RVLNORM3(N, fTmp);
	RVLCOPY3VECTOR(Rj_nj, N);
	
#ifdef RVLVNLOCALIZER_REGISTRATION_V2

	RVLMULMX3X3VECT(jView->cameraView.R0, p_j->P, Rj_pj);					// Rj_pj = Rj * pj
	RVLDIF3VECTORS(jView->cameraView.t0, iView->cameraView.t, T_ij);		// T_ij = tj - ti

	//e_ijk = (Ri*n_ik)' * T_ij - d_ik;
	RVLMULMX3X3VECT(jView->cameraView.R0, p_j->N, Rj_nj);	// Rj_nj = (Rj*n_jk)
	RVLSUM3VECTORS(Ri_ni, Rj_nj, N);
	RVLNORM3(N, fTmp);
	RVLCOPY3VECTOR(Rj_nj, N);

#endif


	double V3Tmp[3];
	RVLDIF3VECTORS(Rj_pj, Ri_pi, V3Tmp);	// V3Tmp = Rj_pj - Ri_pi
	RVLSUM3VECTORS(V3Tmp, T_ij, V3Tmp);		// V3Tmp = V3Tmp + T_ij
	double e_ijk;

#ifdef RVLVNLOCALIZER_REGISTRATION_V2
	e_ijk = RVLDOTPRODUCT3(Rj_nj, V3Tmp);		// e_ijk = N' * V3Tmp
#else
	e_ijk = RVLDOTPRODUCT3(N, V3Tmp);		// e_ijk = N' * V3Tmp
#endif

	// cost calcualtion
	E_ij_ = e_ijk * e_ijk;

	// a - matrix
	double a_ijk_up[3];
	RVLCROSSPRODUCT3(N, Ri_pi, a_ijk_up);				// a_ijk_up = cross((Ri*n_ik), Ri_pi);

	double a_ijk_dwn[3];
	RVLNEGVECT3(N, a_ijk_dwn);							// a_ijk_dwn = -(Ri*n_ik) = -Rn;


	cv::Mat a_up;
	a_up.create(3, 1, CV_64FC1);
	std::memcpy(a_up.data, a_ijk_up, 3 * 1 * sizeof(double));

	cv::Mat a_dwn;
	a_dwn.create(3, 1, CV_64FC1);
	std::memcpy(a_dwn.data, a_ijk_dwn, 3 * 1 * sizeof(double));

	cv::Mat a;												//a_ijk = vertcat(a_ijk_up, a_ijk_dwn);
	a.create(6, 1, CV_64FC1);
	a_up.copyTo(a(cv::Rect(0, 0, 1, 3)));
	a_dwn.copyTo(a(cv::Rect(0, 3, 1, 3)));

	// b - matrix
	double b_ijk_up[3];
	RVLCROSSPRODUCT3(Rj_pj, N, b_ijk_up);				// b_ijk_up = cross((Rj*p_j), N);

	cv::Mat b_up;
	b_up.create(3, 1, CV_64FC1);
	std::memcpy(b_up.data, b_ijk_up, 3 * 1 * sizeof(double));

	cv::Mat b_dwn;
	b_dwn.create(3, 1, CV_64FC1);
	std::memcpy(b_dwn.data, N, 3 * 1 * sizeof(double));

	cv::Mat b;												//b_ijk = vertcat(b_ijk_up, b_ijk_dwn);
	b.create(6, 1, CV_64FC1);
	b_up.copyTo(b(cv::Rect(0, 0, 1, 3)));
	b_dwn.copyTo(b(cv::Rect(0, 3, 1, 3)));

	// g - matrix
	cv::Mat g;
	g.create(6, 1, CV_64FC1);
	g = a * e_ijk;
	double *g_ = (double *)(g.data);

	// h - matrix
	cv::Mat h;
	h.create(6, 1, CV_64FC1);
	h = b * e_ijk;
	double *h_ = (double *)(h.data);

	// A - matrix
	cv::Mat A;
	A.create(6, 6, CV_64FC1);
	A = a * a.t();				// A = a*a';
	double *A_ = (double *)(A.data);

	// B - matrix
	cv::Mat B;
	B.create(6, 6, CV_64FC1);
	B = b * b.t();
	double *B_ = (double *)(B.data);

	// D - matrix
	cv::Mat D;
	D.create(6, 6, CV_64FC1);
	D = a * b.t();
	double *D_ = (double *)(D.data);


	double ni_Ri[3];
	RVLMULMX3X3VECT(iView->cameraView.R, p_i->N, ni_Ri);		// ni_Ri = ni'*Ri'
	d = RVLDOTPRODUCT3(ni_Ri, T_ij);

	//x_nl = d - d_ik;

	std::memcpy(g_ij_, g_, 6 * 1 * sizeof(double));
	std::memcpy(h_ij_, h_, 6 * 1 * sizeof(double));
	std::memcpy(A_ij_, A_, 6 * 6 * sizeof(double));
	std::memcpy(B_ij_, B_, 6 * 6 * sizeof(double));
	std::memcpy(D_ij_, D_, 6 * 6 * sizeof(double));


	return E_ij_;

}

void VNLocalizer::MultiPointCloudCorrection(float *X)
{
	int noViews = Views.size();

	float R0[9];
	float R_[9];
	float t0[3];
	float t_[3];

	cv::Mat x;
	x.create(noViews * 6, 1, CV_32FC1);
	std::memcpy(x.data, X, noViews * 6 * 1 * sizeof(float));

	cv::Mat fi;
	fi.create(3, 1, CV_32FC1);
	float *fi_ = (float *)(fi.data);

	float theta;

	cv::Mat s;
	s.create(3, 1, CV_32FC1);
	float *s_ = (float *)(s.data);

	float u[3];
	float dR[9];

	int goTo_row_fi = 0;
	int goTo_row_s = 3;

	int lamda = 1;

	for (int iView = 0; iView < noViews; iView++)
	{
		x(cv::Rect(0, goTo_row_fi, 1, 3)).copyTo(fi);
		x(cv::Rect(0, goTo_row_s, 1, 3)).copyTo(s);

		if (lamda >= 1)
		{
			theta = sqrt(RVLDOTPRODUCT3(fi_, fi_));

			if (theta >= 1e-6)
			{
				RVLSCALE3VECTOR2(fi_, theta, u);

				AngleAxisToRot<float>(u, theta, dR);

				RVLMXMUL3X3(dR, Views[iView]->cameraView.R, R_);		// R_ = dR * R_

				printf("theta=%f\n", theta);
			}
			else
			{
				RVLCOPYMX3X3(Views[iView]->cameraView.R, R_)
			}

			RVLSUM3VECTORS(Views[iView]->cameraView.t, s_, t_);		// t_ = t_ + s			
		}
		else
		{
			//u = ;
			//psi = ;
			//AngleAxisToRot<float>(u, psi, R_);
		}

		
		GetAngleAxis(R_, u, theta);

		float fTmp;

		RVLNORM3(u, fTmp);

		AngleAxisToRot<float>(u, theta, R_);

		RVLCOPYMX3X3(R_, Views[iView]->cameraView.R);
		RVLCOPY3VECTOR(t_, Views[iView]->cameraView.t)

		goTo_row_fi = goTo_row_fi + 6;
		goTo_row_s = goTo_row_s + 6;
	}
}

void VNLocalizer::MultiPointCloudCorrection(double *X)
{
	// Params.
	double theta_max = 4.0 * DEG2RAD; //deg
	double s_max = 0.03; //m
	//
	int noViews = Views.size();

	double R0[9];
	double R_[9];
	double t0[3];
	double t_[3];

	cv::Mat x;
	x.create(noViews * 6, 1, CV_64FC1);
	std::memcpy(x.data, X, noViews * 6 * 1 * sizeof(double));

	cv::Mat fi;
	fi.create(3, 1, CV_64FC1);
	double *fi_ = (double *)(fi.data);

	double theta;
	double theta_threshold = 5 * DEG2RAD;

	cv::Mat s;
	s.create(3, 1, CV_64FC1);
	double *s_ = (double *)(s.data);

	double u[3];
	double R_k1[9];
	double dR[9];
	double dR_[9];
	RVLUNITMX3(dR_);
	double dt_[3];
	RVLNULL3VECTOR(dt_);

	double average_error = 0.0f;
	double lamda, lamda_s, lamda_fi;
	double uPsi[3], psi[3], theta_psi, fi2, psi2, s2, t2;
	bool bLamda_fi, bLamda_s, bCorrection;

	int goTo_row_fi = 0;
	int goTo_row_s = 3;

	for (int iView = 0; iView < noViews; iView++)
	{
		x(cv::Rect(0, goTo_row_fi, 1, 3)).copyTo(fi);
		x(cv::Rect(0, goTo_row_s, 1, 3)).copyTo(s);


		RVLMXMUL3X3T1(Views[iView]->cameraView.R, Views[iView]->cameraView.R0, R_k1);		// R_k1 = R_k-1' * R0

		fi2 = RVLDOTPRODUCT3(fi_, fi_);

		if (RotToAngleAxis<double>(R_k1, uPsi, theta_psi))
		{
			// Get lamda_fi
			double tmp, psi_[3];

			RVLSCALE3VECTOR(uPsi, theta_psi, psi);

			if (bLamda_fi = (fi2 > 1e-10))
			{
				psi2 = theta_psi * theta_psi;
				tmp = theta_max * theta_max - psi2;					// tmp = theta_max^2 - ||psi||^2
				if (tmp < 0.0)
					tmp = 0.0f;
				tmp = fi2 * tmp;									// tmp =  ||fi||^2 * (theta_max^2 - ||psi||^2)
				tmp = RVLDOTPRODUCT3(psi, fi_)*RVLDOTPRODUCT3(psi, fi_) + tmp;			// tmp = (psi' * fi)^2 + ||fi||^2 * (theta_max^2 - ||psi||^2)
				tmp = sqrt(tmp);
				RVLNEGVECT3(psi, psi_);
				tmp = RVLDOTPRODUCT3(psi_, fi_) + tmp;									// tmp = -psi'*fi + sqrt((psi' * fi)^2 + ||fi||^2 * (theta_max^2 - ||psi||^2))
				lamda_fi = tmp / fi2;
			}

			// Get lamda_s
			double tmp2, tk_[3];
			s2 = RVLDOTPRODUCT3(s_, s_);

			if (bLamda_s = (s2 > 1e-10))
			{
				t2 = RVLDOTPRODUCT3(Views[iView]->cameraView.t, Views[iView]->cameraView.t);
				tmp2 = s_max * s_max - t2;					// tmp2 = s_max^2 - ||tk||^2
				if (tmp2 < 0.0)
					tmp2 = 0.0f;
				tmp2 = s2 * tmp2;																			// tmp2 =  ||s||^2 * (s_max^2 - ||tk||^2)
				tmp2 = RVLDOTPRODUCT3(Views[iView]->cameraView.t, s_)*RVLDOTPRODUCT3(Views[iView]->cameraView.t, s_) + tmp2;	// tmp2 = (tk' * s)^2 + ||s||^2 * (s_max^2 - ||tk||^2)
				tmp2 = sqrt(tmp2);
				RVLNEGVECT3(Views[iView]->cameraView.t, tk_);
				tmp2 = RVLDOTPRODUCT3(tk_, s_) + tmp2;																			// tmp2 = -tk'*s + sqrt((tk' * s)^2 + ||s||^2 * (s_max^2 - ||tk||^2))
				lamda_s = tmp2 / s2;
			}

			if (bCorrection = (bLamda_fi || bLamda_s))
			{
				if (!bLamda_fi)
					lamda = lamda_s;
				else if (!bLamda_s)
					lamda = lamda_fi;
				else
					// Get lamda = min(lamda_fi, lamda_s)
					lamda = RVLMIN(lamda_fi, lamda_s);
			}
		}
		else
		{
			lamda = 1;

			bCorrection = true;
		}

		bCorrection = true;
		lamda = 2.0f;

		if (bCorrection)
		{
			if (lamda >= 1)
			{
				theta = sqrt(fi2);		// theta = ||fi||

				if (theta >= 1e-6)
				{
					//if (theta >= theta_threshold)
					//{
					//	theta = theta_threshold;
					//}

					RVLSCALE3VECTOR2(fi_, theta, u);

					AngleAxisToRot<double>(u, theta, dR);

					//RVLMXMUL3X3(dR, Views[iView]->cameraView.R, R_);		// R_ = dR * R_
					//printf("theta=%f\n", theta);
				}
				else
				{
					//RVLCOPYMX3X3(Views[iView]->cameraView.R, R_)
					RVLUNITMX3(dR);
				}
				//RVLCOMPTRANSF3D(dR, s_, dR_, dt_, R_, t_);
				//RVLCOPYMX3X3(R_, dR_);
				//RVLCOPY3VECTOR(t_, dt_);

				RVLCOMPTRANSF3D(dR, s_, Views[iView]->cameraView.R, Views[iView]->cameraView.t, R_, t_);
				//RVLCOMPTRANSF3D(dR_, dt_, Views[iView]->cameraView.R, Views[iView]->cameraView.t, R_, t_);

				//RVLMULMX3X3VECT(dR, Views[iView]->cameraView.t, t_);	// t_ = dR * t
				//RVLSUM3VECTORS(t_, s_, t_);		// t_ = t_ + s			

				RotToAngleAxis<double>(R_, u, theta);

				double fTmp;

				RVLNORM3(u, fTmp);

				AngleAxisToRot<double>(u, theta, R_);

				printf("view %d, theta=%f\n", iView, theta);

				RVLCOPYMX3X3(R_, Views[iView]->cameraView.R);
				RVLCOPY3VECTOR(t_, Views[iView]->cameraView.t)
			}
			else if (lamda > 1e-6)
			{
				double lamdaFi[3], lamdaS[3];
				RVLSCALE3VECTOR(fi_, lamda, lamdaFi);
				double newPsi[3];
				RVLSUM3VECTORS(psi, lamdaFi, newPsi);

				theta = sqrt(RVLDOTPRODUCT3(newPsi, newPsi));
				RVLSCALE3VECTOR2(newPsi, theta, newPsi);

				printf("view %d, theta=%f\n", iView, theta);

				AngleAxisToRot<double>(newPsi, theta, dR);

				RVLMXMUL3X3(dR, Views[iView]->cameraView.R0, R_);
				RVLCOPYMX3X3(R_, Views[iView]->cameraView.R);

				RVLSCALE3VECTOR(s_, lamda, lamdaS);
				RVLSUM3VECTORS(Views[iView]->cameraView.t, lamdaS, Views[iView]->cameraView.t);

			}
		}

		goTo_row_fi = goTo_row_fi + 6;
		goTo_row_s = goTo_row_s + 6;
	} // end iView

	average_error = average_error / noViews;
	printf("Average theta = %f\n", average_error);
}



float VNLocalizer::MultiPointCloudToPlaneError()
{
	int noViews = Views.size();

	// Projection Cube properties
	int cubeHalfSize = cube.cubeHalfSize;
	int noCubeSideRows = 2 * cubeHalfSize + 1;
	int noCubeSideColumns = 2 * cubeHalfSize + 1;

	float CosAngle = cube.CosAngle;
	float distance_threshold = cube.distance_threshold;

	float t_[3];
	float nR[3];
	float d;

	float T_ij[3];
	float Rj_pj[3];

	float Ri_ni[3];
	float e_ijk;

	float E_ij_;
	float E_ij_sum = 0;

	int lView;

	bool *bAssociated = new bool[noViews];

	memset(bAssociated, 0, noViews * sizeof(bool));

	for (int kView = 0; kView < noViews; kView++)
	{
		int noOverlappingViews = Views[kView]->overlapingViewList.size();

		for (int l = 0; l < noOverlappingViews; l++)
		{
			lView = Views[kView]->overlapingViewList[l];

			if (lView < kView && !bAssociated[kView])
			{
				int noCorrespondences = Views[kView]->correspodance_list[l].size();

				for (int iCrsp = 0; iCrsp < noCorrespondences; iCrsp++)
				{
					Pair<int, int> crspPts;
					crspPts = Views[kView]->correspodance_list[l].at(iCrsp);

					int ptIndex_iView = crspPts.a;
					int ptIndex_jView = crspPts.b;

					Point *p_i = &(Views[kView]->mesh.NodeArray.Element[ptIndex_iView]);
					Point *p_j = &(Views[lView]->mesh.NodeArray.Element[ptIndex_jView]);

					//				float pt_distance[3];
					//				float e_ik, d_ik;
					//
					//				//RVLDIF3VECTORS(p_j->P, p_i->P, pt_distance);
					//				//e_ik = RVLDOTPRODUCT3(p_i->N, pt_distance);
					//				d_ik = RVLDOTPRODUCT3(p_i->N, p_i->P);
					//
					//				//d_ik = abs(d_ik);
					//
					//				//T_ij = Rj*p_j + tj - ti;
					//#ifdef RVLVNLOCALIZER_REGISTRATION_V2
					//				RVLMULMX3X3VECT(Views[lView]->cameraView.R0, p_j->P, Rj_pj);		// Rj_pj = Rj * pj
					//				RVLSUM3VECTORS(Rj_pj, Views[lView]->cameraView.t0, t_);			// t_ = Rj_pj + tj
					//#else
					//				RVLMULMX3X3VECT(Views[lView]->cameraView.R, p_j->P, Rj_pj);		// Rj_pj = Rj * pj
					//				RVLSUM3VECTORS(Rj_pj, Views[lView]->cameraView.t, t_);			// t_ = Rj_pj + tj
					//#endif
					//				RVLTRANSF3(p_i->P, Views[kView]->cameraView.R, Views[kView]->cameraView.t, T_ij);
					//
					//				RVLDIF3VECTORS(t_, Views[kView]->cameraView.t, T_ij);			// T_ij = t_ - ti
					//
					//				//e_ijk = (Ri*n_ik)' * T_ij - d_ik;
					//				RVLMULMX3X3VECT(Views[kView]->cameraView.R, p_i->N, Ri_ni);		// Ri_ni = (Ri*n_ik)
					//				e_ijk = RVLDOTPRODUCT3(Ri_ni, T_ij);							// e_ijk = Rn' * T_ij
					//				e_ijk = e_ijk - d_ik;


					double Ri_pi[3];
					RVLMULMX3X3VECT(Views[kView]->cameraView.R, p_i->P, Ri_pi);					// Ri_pi = Ri * pi

					double Rj_nj[3];

#ifdef RVLVNLOCALIZER_REGISTRATION_V2
					RVLMULMX3X3VECT(Views[lView]->cameraView.R0, p_j->P, Rj_pj);					// Rj_pj = Rj * pj
					RVLDIF3VECTORS(Views[lView]->cameraView.t0, Views[kView]->cameraView.t, T_ij);		// T_ij = tj - ti

					//e_ijk = (Ri*n_ik)' * T_ij - d_ik;
					RVLMULMX3X3VECT(Views[lView]->cameraView.R0, p_j->N, Rj_nj);	// Rj_nj = (Rj*n_jk)
#else
					RVLMULMX3X3VECT(Views[lView]->cameraView.R, p_j->P, Rj_pj);					// Rj_pj = Rj * pj
					RVLDIF3VECTORS(Views[lView]->cameraView.t, Views[kView]->cameraView.t, T_ij);		// T_ij = tj - ti

					//e_ijk = (Ri*n_ik)' * T_ij - d_ik;
					RVLMULMX3X3VECT(Views[lView]->cameraView.R, p_j->N, Rj_nj);	// Rj_nj = (Rj*n_jk)
#endif

					double V3Tmp[3];
					RVLDIF3VECTORS(Rj_pj, Ri_pi, V3Tmp);	// V3Tmp = Rj_pj - Ri_pi
					RVLSUM3VECTORS(V3Tmp, T_ij, V3Tmp);		// V3Tmp = V3Tmp + T_ij
					double e_ijk;

#ifdef RVLVNLOCALIZER_REGISTRATION_V2
					e_ijk = RVLDOTPRODUCT3(Rj_nj, V3Tmp);		// e_ijk = N' * V3Tmp
#else
					//e_ijk = RVLDOTPRODUCT3(N, V3Tmp);		// e_ijk = N' * V3Tmp
					e_ijk = 0;
#endif
					// cost calcualtion
					E_ij_ = e_ijk * e_ijk;

					E_ij_sum = E_ij_sum + E_ij_;

				}	// end for iCrsp	

				bAssociated[kView] = true;
			}
		}	// end for lView

	}	// end for kView

	delete[] bAssociated;

	return E_ij_sum;

}


void VNLocalizer::MultiViewRegistration(int noIterations, bool visualizeRoom, bool visualizeCorrespondences, char *resultsFolder)
{
	int noViews = Views.size();
	
	float E_new;
	double E;
	float expectedCostChange;
	float costChange;

	//float *X = new float[6 * noViews];
	double *X = new double[6 * noViews];

	for (int iteration = 0; iteration < noIterations; iteration++)
	{
		// Create Crsps

		// Clear cashe
		for (int iView = 0; iView < noViews; iView++)
		{
			Views[iView]->O_k_list.clear();
			Views[iView]->C_k_list.clear();
			Views[iView]->correspodance_list.clear();
			//Views[iView]->ClearAmbiguousPoints(rotErrorBound, camera);
		}
		ImagesSampling();
		DataAssociation();
		

		// Visualization.
		//if (visualizeCorrespondences)
		if (iteration % 10 == 0 && visualizeCorrespondences)
		{
			printf("\n Visualizing: \n");
			Visualize();
		}

		if (iteration == 0 && visualizeRoom)
		{
			VisualizeMergedViews();
		}

#ifdef RVLVNLOCALIZER_LOG_RESULTS_SCENESEQUENCE
		//Save Rt
		if (iteration % 10 == 0 && iteration != 0)
		{
			for (int iView = 0; iView < noViews; iView++)
			{
				RVLCOPYMX3X3(Views[iView]->cameraView.R, Views[iView]->cameraView.R_tmp);
				RVLCOPY3VECTOR(Views[iView]->cameraView.t, Views[iView]->cameraView.t_tmp);
			}

			FinalPoseCorrection();

			//FILE *fpT = fopen("D:\\Mateja\\Results\\Replica\\R_corrected.txt", "w");
			//fclose(fpT);

			//fpT = fopen("D:\\Mateja\\Results\\Replica\\t_corrected.txt", "w");
			//fclose(fpT);

			FILE *pRCorrectedFile = fopen((std::string(resultsFolder) + "\\" + std::to_string(iteration) + "\\R_corrected.txt").data(), "w");
			fclose(pRCorrectedFile);

			FILE *ptCorrectedFile = fopen((std::string(resultsFolder) + "\\" + std::to_string(iteration) + "\\t_corrected.txt").data(), "w");
			fclose(ptCorrectedFile);

			for (int iView = 0; iView < noViews; iView++)
			{

				//fpT = fopen("D:\\Mateja\\Results\\Replica\\R_corrected.txt", "a");
				//PrintMatrix<float>(fpT, Views[iView]->cameraView.R, 3, 3);
				//fclose(fpT);

				//fpT = fopen("D:\\Mateja\\Results\\Replica\\t_corrected.txt", "a");
				//PrintMatrix<float>(fpT, Views[iView]->cameraView.t, 3, 1);
				//fclose(fpT);

				pRCorrectedFile = fopen((std::string(resultsFolder) + "\\" + std::to_string(iteration) + "\\R_corrected.txt").data(), "a");
				PrintMatrix<float>(pRCorrectedFile, Views[iView]->cameraView.R, 3, 3);
				fclose(pRCorrectedFile);

				ptCorrectedFile = fopen((std::string(resultsFolder) + "\\" + std::to_string(iteration) + "\\t_corrected.txt").data(), "a");
				PrintMatrix<float>(ptCorrectedFile, Views[iView]->cameraView.t, 3, 1);
				fclose(ptCorrectedFile);

			}

			int debug = 1;

			for (int iView = 0; iView < noViews; iView++)
			{
				RVLCOPYMX3X3(Views[iView]->cameraView.R_tmp, Views[iView]->cameraView.R);
				RVLCOPY3VECTOR(Views[iView]->cameraView.t_tmp, Views[iView]->cameraView.t);
			}
		}

#endif

		// ICP
		printf("\nIteration: %d\n", iteration + 1);

		E = MultiPointCloudReg(X);
		MultiPointCloudCorrection(X);			// Correct R, t for every view

		E_new = MultiPointCloudToPlaneError();
		costChange = E_new - E;
		printf("Cost change = %.4f\n\n", costChange);

	}

#ifdef RVLVNLOCALIZER_REGISTRATION_V2
	FinalPoseCorrection();
#endif

	if (visualizeRoom)
	{
		VisualizeMergedViews();
	}

	delete[] X;


}

void VNLocalizer::FinalPoseCorrection()
{
	int noViews = Views.size();

	bool *bAssociated = new bool[noViews];

	memset(bAssociated, 0, noViews * sizeof(bool));

	int lView;
	float R_[9], t_[3], tmp3x1[3];

	for (int kView = 0; kView < noViews; kView++)
	{
		int noOverlappingViews = Views[kView]->overlapingViewList.size();

		for (int l = 0; l < noOverlappingViews; l++)
		{
			lView = Views[kView]->overlapingViewList[l];

			if (lView < kView && !bAssociated[kView])
			{
				RVLCOMPTRANSF3DWITHINV(Views[lView]->cameraView.R0, Views[lView]->cameraView.t0, Views[kView]->cameraView.R, Views[kView]->cameraView.t, R_, t_, tmp3x1);		// R_ = inv(R_l0) * Rk
				RVLCOMPTRANSF3D(Views[lView]->cameraView.R, Views[lView]->cameraView.t, R_, t_, Views[kView]->cameraView.R, Views[kView]->cameraView.t);						// Rk = Rl * R_
				bAssociated[kView] = true;
			}

		}
	}

}


void VNLocalizer::Visualize()
{
	int noViews = Views.size();

	LOCAL::DisplayCallbackData displayData;

	displayData.pVisualizer = pVisualizer;

	double color1[3] = {0.0f, 1.0f, 0.0};
	double color2[3] = { 1.0f, 1.0f, 0.0f};

	int l, lView;

	for (int kView = 0; kView < noViews; kView++)
	{
		int noOverlappingViews = Views[kView]->overlapingViewList.size();

		for (l = 0; l < noOverlappingViews; l++)
		{
			lView = Views[kView]->overlapingViewList[l];

			printf("%d with %d\n", kView, lView);

			pVisualizer->AddMesh(Views[kView]->mesh.pPolygonData, color1, Views[kView]->cameraView.R, Views[kView]->cameraView.t);

#ifdef RVLVNLOCALIZER_REGISTRATION_V2
			pVisualizer->AddMesh(Views[lView]->mesh.pPolygonData, color2, Views[lView]->cameraView.R0, Views[lView]->cameraView.t0);
#else
			pVisualizer->AddMesh(Views[lView]->mesh.pPolygonData, color2, Views[lView]->cameraView.R, Views[lView]->cameraView.t);
#endif


			VisualizeAmbiguousPoints(pVisualizer, Views[kView]);
			VisualizeAmbiguousPoints(pVisualizer, Views[lView]);

			VisualizeCorrespondences(pVisualizer, Views[kView], Views[lView], Views[kView]->correspodance_list[l]);
			//VisualizeCrspPoints(pVisualizer, Views[kView], Views[lView], Views[kView]->correspodance_list[l]);
			VisualizeSamplePoints(pVisualizer, kView, lView);

			pVisualizer->SetMouseRButtonDownCallback(LOCAL::MouseRButtonDown, &displayData);

			pVisualizer->Run();

			pVisualizer->renderer->RemoveAllViewProps();
		}
	}
}

void VNLocalizer::VisualizeCrspPoints(Visualizer *pVisualizer, View *kView, View *lView, std::vector<Pair<int, int>> correspondences)
{
	float P[3];
	Point *pPt_vw;
	int ptID_vw;

	int noCrsp = correspondences.size();

	Array<Point> crspPoints_kView;
	crspPoints_kView.Element = new Point[noCrsp];
	crspPoints_kView.n = noCrsp;
	Point *pPt_kView = crspPoints_kView.Element;

	Array<Point> crspPoints_lView;
	crspPoints_lView.Element = new Point[noCrsp];
	crspPoints_lView.n = noCrsp;
	Point *pPt_lView = crspPoints_lView.Element;

	for (int i = 0; i < noCrsp; i++, pPt_kView++, pPt_lView++)
	{
		ptID_vw = correspondences[i].a;
		pPt_vw = kView->mesh.NodeArray.Element + ptID_vw;
		RVLTRANSF3(pPt_vw->P, kView->cameraView.R, kView->cameraView.t, P);
		RVLCOPY3VECTOR(P, pPt_kView->P);

		ptID_vw = correspondences[i].b;
		pPt_vw = lView->mesh.NodeArray.Element + ptID_vw;
		RVLTRANSF3(pPt_vw->P, lView->cameraView.R, lView->cameraView.t, P);
		RVLCOPY3VECTOR(P, pPt_lView->P);
	}

	unsigned char color[3];

	RVLSET3VECTOR(color, 0, 255, 0);

	pVisualizer->DisplayPointSet<float, Point>(crspPoints_kView, color, 4.0f);
	pVisualizer->DisplayPointSet<float, Point>(crspPoints_lView, color, 4.0f);


	delete[] crspPoints_kView.Element;
	delete[] crspPoints_lView.Element;
}

void VNLocalizer::VisualizeSamplePoints(Visualizer *pVisualizer, int kView, int lView)
{
	int noOverlappingViews = Views[kView]->overlapingViewList.size();

	if (noOverlappingViews < 1)
		return;

	Array<Point> points;

	points.Element = NULL;

	int i, l;
	float *PSrc, *PTgt;

	for (l = 0; l < noOverlappingViews; l++)
	{
		if (lView == Views[kView]->overlapingViewList[l])
		{
			points.Element = new Point[Views[kView]->C_k_list[l].size()];

			points.n = 0;

			for (i = 0; i < Views[kView]->C_k_list[l].size(); i++)
			{
				if (Views[kView]->C_k_list[l][i].ptID_jView == 30)
					int debug = 1;

				PSrc = Views[kView]->mesh.NodeArray.Element[Views[kView]->C_k_list[l][i].ptID_iView].P;

				PTgt = points.Element[points.n++].P;

				RVLTRANSF3(PSrc, Views[kView]->cameraView.R, Views[kView]->cameraView.t, PTgt);
			}
		}
	}

	unsigned char color[3];

	RVLSET3VECTOR(color, 0, 255, 0);

	pVisualizer->DisplayPointSet<float, Point>(points, color, 4.0f);

	RVL_DELETE_ARRAY(points.Element);
}

void VNLocalizer::VisualizeAmbiguousPoints(Visualizer *pVisualizer, View *iView)
{
	float P[3];
	Point *pPt_vw;
	int ptID_vw;

	Array<int> ambiguousPoints;
	ambiguousPoints.Element = new int[iView->mesh.NodeArray.n];
	ambiguousPoints.n = 0;

	for (int iPoint = 0; iPoint < iView->mesh.NodeArray.n; iPoint++)
	{
		Point *pPt = iView->mesh.NodeArray.Element + iPoint;
		if (pPt->bAmbiguous)
			ambiguousPoints.Element[ambiguousPoints.n++] = iPoint;
	}

	unsigned char color[3];

	RVLSET3VECTOR(color, 128, 128, 255);

	pVisualizer->PaintPointSet(&ambiguousPoints, iView->mesh.pPolygonData, color);

	delete[] ambiguousPoints.Element;
}

void VNLocalizer::VisualizeMergedViews()
{
	int noViews = Views.size();

	double color[] = { 0.0f, 1.0f, 0.0f };

	int l, lView;

	for (int kView = 0; kView < noViews; kView++)
	{
		
		pVisualizer->AddMesh(Views[kView]->mesh.pPolygonData, color, Views[kView]->cameraView.R, Views[kView]->cameraView.t);
	}

	pVisualizer->Run();

	pVisualizer->renderer->RemoveAllViewProps();
}

void VNLocalizer::VisualizeCentralCells()
{
	//pVisualizer->AddMesh(Views[lView]->mesh.pPolygonData, color2, Views[lView]->cameraView.R, Views[lView]->cameraView.t);

	//VisualizeAmbiguousPoints(pVisualizer, Views[kView]);
}


void LOCAL::MouseRButtonDown(vtkObject* caller, unsigned long eid, void* clientdata, void *calldata)
{
	vtkSmartPointer<vtkRenderWindowInteractor> interactor = reinterpret_cast<vtkRenderWindowInteractor*>(caller);
	LOCAL::DisplayCallbackData *pData = (LOCAL::DisplayCallbackData *)clientdata;

	//Mesh *pMesh = pData->pMesh;

	//vtkSmartPointer<vtkPolyData> pd = pMesh->pPolygonData;

	//vtkSmartPointer<vtkFloatArray> pointData;
	//vtkSmartPointer<vtkUnsignedCharArray> rgbPointData;
	//vtkSmartPointer<vtkFloatArray> normalPointData;
	//int noPts = 0;
	//FetchVTKPointData(pd, pointData, rgbPointData, normalPointData, noPts);

	pData->pVisualizer->pointPicker->Pick(interactor->GetEventPosition()[0], interactor->GetEventPosition()[1], 0,
		interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
	vtkIdType selectedPoint = pData->pVisualizer->pointPicker->GetPointId();

	printf("selected point = %d\n", selectedPoint);
}
