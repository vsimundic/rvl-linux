#define _CRT_SECURE_NO_WARNINGS

#include "RVLCore2.h"
#include "RVLVTK.h"
#include "Util.h"
#include "Space3DGrid.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
#include "SurfelGraph.h"
#include "ProjectionCube.h"
#include "View.h"
//#include "RVLUtil.h"
#include <errno.h>

using namespace RVL;
using namespace LOCAL;

//Function definitions

View::View()
{
	//inicijalizirati vrijednosti i memoriju
	surfels.NodeArray.Element = NULL;
	mesh.NodeArray.Element = NULL;
	fileName = NULL;

	// Image properties
	image_width = 320;
	image_height = 240;

	// Sample image to cells
	// Set size of one cell
	image_cell_size_width = 16;
	image_cell_size_height = 16;
}


View::~View()
{
	Clear();
}


void View::Create(char *fileName_, Mesh *pMesh, SurfelGraph *pSurfels, CameraView cameraView_)
{ 
	int minSurfelSize = 1000;

	fileName = RVLCreateString(fileName_);

	//pCameraView = pCameraView_;
	cameraView = cameraView_;

	mesh.NodeArray.n = pMesh->NodeArray.n;
	surfels.NodeArray.n = pSurfels->NodeArray.n;

	mesh.NodeArray.Element = new Point[pMesh->NodeArray.n];
	surfels.NodeArray.Element = new Surfel[pSurfels->NodeArray.n];

	memcpy(mesh.NodeArray.Element, pMesh->NodeArray.Element, sizeof(Point) * pMesh->NodeArray.n);
	memcpy(surfels.NodeArray.Element, pSurfels->NodeArray.Element, sizeof(Surfel) * pSurfels->NodeArray.n);

	surfels.PtMem = new QLIST::Index2[pMesh->NodeArray.n];

	QLIST::Index2 *pPtIdxTgt = surfels.PtMem;

	int iSurfel;

	for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++)
	{
		QList<QLIST::Index2> *pPtListTgt = &(surfels.NodeArray.Element[iSurfel].PtList);

		RVLQLIST_INIT(pPtListTgt);

		QList<QLIST::Index2> *pPtListSrc = &(pSurfels->NodeArray.Element[iSurfel].PtList);

		QLIST::Index2 *pPtIdxSrc = pPtListSrc->pFirst;	// pointer to the first element of pPtListSrc

		while (pPtIdxSrc)
		{
			RVLQLIST_ADD_ENTRY(pPtListTgt, pPtIdxTgt);

			pPtIdxTgt->Idx = pPtIdxSrc->Idx;

			pPtIdxTgt++;

			pPtIdxSrc = pPtIdxSrc->pNext;
		}
	}

	//Creating surfel pairs in the current view w.r.t. angle between two surfels
	float CosAngle = cos(45.00 * PI / 180.0);

	for (int surfelIndex = 0; surfelIndex < surfels.NodeArray.n - 1; surfelIndex++)
	{
		Surfel *pCurrentSurfel = &surfels.NodeArray.Element[surfelIndex];

		if (pCurrentSurfel->size < minSurfelSize)
			continue;

		for (int CandidateSurfelIndex = surfelIndex + 1; CandidateSurfelIndex < surfels.NodeArray.n - 1; CandidateSurfelIndex++)
		{
			Surfel *pCandidateSurfel = &surfels.NodeArray.Element[CandidateSurfelIndex];
			float CosAngleBetweenSurfels = 0.0;
			//float DotProductN, N1_abs, N2_abs;

			if (pCandidateSurfel->size < minSurfelSize)
				continue;

			CosAngleBetweenSurfels = RVLDOTPRODUCT3(pCurrentSurfel->N, pCandidateSurfel->N);
			if (CosAngleBetweenSurfels < CosAngle)
			{
				surfelPair *surfelPair_temp = new surfelPair;

				surfelPair_temp->S1_ID = surfelIndex;
				surfelPair_temp->S2_ID = CandidateSurfelIndex;

				surfelPairs_list.push_back(surfelPair_temp);
			}

		}
	}

}



void View::Load(Mesh *pMesh)
{
	mesh.NodeArray.n = pMesh->NodeArray.n;
	mesh.NodeArray.Element = new Point[pMesh->NodeArray.n];
	memcpy(mesh.NodeArray.Element, pMesh->NodeArray.Element, sizeof(Point) *  pMesh->NodeArray.n);
}

void View::InformationContnet()
{
	// trebaju mi sve tockiice iz jedne celije
	// svaki view treba imati niz celija[9] u koji ce biti upisana kovarijancna matrica

	// Get total number of cells in image
	noColumnsImage = image_width / image_cell_size_width;			// = 20;
	noRowsImage = image_height / image_cell_size_height;			// = 15;

	// Initialize covariance_list
	covariance_list = new float[9 * noColumnsImage * noRowsImage];
	
	//covariance_list.w = noColumnsImage;
	//covariance_list.h = noRowsImage;

	float *covarince_cell;

	for (int image_row = 0; image_row < noRowsImage; image_row++)
	{
		for (int image_column = 0; image_column < noColumnsImage; image_column++)
		{
			
			// Calculate covariance of a current cell
			covarince_cell = covariance_list + (image_column + image_row * noColumnsImage) * 9;

			RVLNULLMX3X3(covarince_cell);

			for (int cell_row = 0; cell_row < image_cell_size_height; cell_row++)
			{
			
				for (int cell_column = 0; cell_column < image_cell_size_width; cell_column++)
				{
					// Get point index 
					int pointIndex = (cell_row + image_row * image_cell_size_height) * image_width + (cell_column + image_column * image_cell_size_width);
					
					// Get point
					Point currentPoint = mesh.NodeArray.Element[pointIndex];

					if (currentPoint.bValid == false)
						continue;

					float N_lenght = RVLDOTPRODUCT3(currentPoint.N, currentPoint.N);
					if (N_lenght < 0.9)
						continue;
					
					float covariance_point[9];
					RVLVECTCOV3(currentPoint.N, covariance_point);
					RVLSUMMX3X3UT(covariance_point, covarince_cell, covarince_cell);
				}
			}
			
		}

	}
}

void View::Clear()
{
	//funkcija za brisanje
	RVL_DELETE_ARRAY(surfels.NodeArray.Element);
	RVL_DELETE_ARRAY(mesh.NodeArray.Element);
	RVL_DELETE_ARRAY(fileName);

}


////////////////////////////////////////////////
//
//     GLOBAL FUNCTIONS
//
////////////////////////////////////////////////

int LOCAL::LoadViews(char *viewFileName, std::vector<CameraView>* cameraViews)
{
	FILE* pCameraFile = NULL;
	pCameraFile = fopen(viewFileName, "r");

	char msg[500] = {};
	char* ptr = &msg[0];
	ptr = strerror(errno);

	if (pCameraFile == NULL)
	{
		printf("Cameras file not found.");
		return -1;
	}

	CameraView CameraView;

	CameraView.iCameraView = 0;

	RVLNULLMX3X3(CameraView.R);

	float xf = 0.0;
	float yf = 0.0;
	float value = 0.0;

	while (!feof(pCameraFile))
	{
		fscanf(pCameraFile, "%f %f %f %f %f %f %f %f %f %f %f %f\n", &CameraView.t[0], &CameraView.t[1], &CameraView.t[2], &CameraView.z[0], &CameraView.z[1], &CameraView.z[2], &CameraView.y[0], &CameraView.y[1], &CameraView.y[2], &xf, &yf, &value);

		//change direction of y-axis
		RVLNEGVECT3(CameraView.y, CameraView.y);

		//Calcualting x-axis --> Creating complete R
		RVLCROSSPRODUCT3(CameraView.y, CameraView.z, CameraView.x);


		CameraView.R[0] = CameraView.x[0];
		CameraView.R[1] = CameraView.y[0];
		CameraView.R[2] = CameraView.z[0];
		CameraView.R[3] = CameraView.x[1];
		CameraView.R[4] = CameraView.y[1];
		CameraView.R[5] = CameraView.z[1];
		CameraView.R[6] = CameraView.x[2];
		CameraView.R[7] = CameraView.y[2];
		CameraView.R[8] = CameraView.z[2];

	/*	int n = 0;
		for (int i = 0; i < 0; i++)
		{
			CameraView.R[i] = CameraView.x[n];
			CameraView.R[i+1] = CameraView.y[n];
			CameraView.R[i+2] = CameraView.z[n];
			i = i + 2;
			n++;
		}*/
		

		RVLHTRANSFMX(CameraView.R, CameraView.t, CameraView.T);

		cameraViews->push_back(CameraView);
		//ili? cameraViews.push_back(CameraView);

		CameraView.iCameraView++;
	}


	fclose(pCameraFile);
	return 0;


	
	//Reading camera files from SUNCG txt file
	// 
	// vc --> t
	// tc --> z
	// uc --> -y
	//
	//fscanf(pCameraFile, "%f %f %f %f %f %f %f %f %f %f %f %f\n", &CameraView1.vc.x[0], &CameraView1.vc.y[1], &CameraView1.vc.z[2], &CameraView1.tc.x[0], &CameraView1.tc.y[1], &CameraView1.tc.z[2], &CameraView1.uc.x[0], &CameraView1.uc.y[1], &CameraView1.uc.z[2], &xf, &yf, &value);
	//fscanf(pCameraFile, "%f %f %f %f %f %f %f %f %f %f %f %f\n", &CameraView2.vc.x[0], &CameraView2.vc.y[1], &CameraView2.vc.z[2], &CameraView2.tc.x[0], &CameraView2.tc.y[1], &CameraView2.tc.z[2], &CameraView2.uc.x[0], &CameraView2.uc.y[1], &CameraView2.uc.z[2], &xf, &yf, &value);

	//IZ SUNCG TOOLBOX-a
	// Read file
	/*
	RNScalar vx, vy, vz, tx, ty, tz, ux, uy, uz, xf, yf, value;
	while (fscanf(fp, "%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf", &vx, &vy, &vz, &tx, &ty, &tz, &ux, &uy, &uz, &xf, &yf, &value) == (unsigned int)12) {
	R3Point viewpoint(vx, vy, vz);
	R3Vector towards(tx, ty, tz);
	R3Vector up(ux, uy, uz);
	R3Vector right = towards % up;
	towards.Normalize();
	up = right % towards;
	up.Normalize();
	R3Camera *camera = new R3Camera(viewpoint, towards, up, xf, yf, neardist, fardist);
	camera->SetValue(value);
	cameras->Insert(camera);
	}
	*/

}

int CameraView::LoadCameraFile(char *filePath)
{
	float temp;
	char temp2;
	float PanAngle, TiltAngle, RollAngle;
	
	char *cameraFileName = RVLCreateFileName(filePath, ".ply", -1, "-O.txt");

	FILE* pCameraFile = NULL;
	pCameraFile = fopen(cameraFileName, "r");

	delete[] cameraFileName;

	char msg[500] = {};
	char* ptr = &msg[0];
	ptr = strerror(errno);

	if (pCameraFile == NULL)
	{
		printf("Camera file not found.");
		return -1;
	}
	else
	{
		fscanf(pCameraFile, "%f %f %f %f %f %f %f %c\n", &temp, &temp, &temp, &PanAngle, &TiltAngle, &RollAngle, &temp, &temp2);
	}

	fclose(pCameraFile);

#ifdef MATEJA
	PanTiltRoll<float>(PanAngle, TiltAngle, RollAngle, R);
#else
	RVLUNITMX3(R);
#endif
	RVLNULL3VECTOR(t);

	// Write R, t to file
	//char *viewFileName = RVLCreateFileName(filePath, ".ply", -1, "_camera.txt");
	//pCameraFile = fopen(viewFileName, "w");

	//fprintf(pCameraFile, "%f %f %f %f %f %f %f %f %f ", R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8]);
	//fprintf(pCameraFile, "%f %f %f\n", t[0], t[1], t[2]);

	//fclose(pCameraFile);

	//delete[] viewFileName;

	return 0;
}


