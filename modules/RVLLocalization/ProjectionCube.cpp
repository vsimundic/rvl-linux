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
#include <errno.h>


using namespace RVL;
//using namespace LOCAL;

ProjectionCube::ProjectionCube()
{
	//inicijalizirati vrijednosti i memoriju
	transformedPoints.Element = NULL;
}

ProjectionCube::~ProjectionCube()
{
	Clear();
}

void ProjectionCube::CreateProjectionCube(int cubeHalfSizeIn, float CosAngle_in, float distance_threshold_in)
{
	cubeHalfSize = cubeHalfSizeIn;
	CosAngle = CosAngle_in;
	distance_threshold = distance_threshold_in;

	int noCubeSideRows = 2 * cubeHalfSize + 1;
	int noCubeSideColumns = 2 * cubeHalfSize + 1;

	float x[3], y[3], z[3];
	int id;
	float *R;

	for (int i = 0; i < 6; i++)
	{		
		RVLNULL3VECTOR(z);		
		id = (i % 3);
		z[id] = 1;
		if (i > 2)
		{
			RVLNEGVECT3(z, z);
		}

		RVLNULL3VECTOR(x);
		id = ((i+1) % 3);
		x[id] = 1;
		if (i > 2)
		{
			RVLNEGVECT3(x, x);
		}

		RVLCROSSPRODUCT3(z, x, y);

		R = RCube[i];

		RVLCOPYTOCOL3(x, 0, R);
		RVLCOPYTOCOL3(y, 1, R);
		RVLCOPYTOCOL3(z, 2, R);

		cubeMatrix[i] = new int[noCubeSideRows * noCubeSideColumns];
	}

	for (int i = 0; i < 6; i++)
		memset(cubeMatrix[i], 0, sizeof(int) * noCubeSideRows * noCubeSideColumns);		//pre-set all cells to 0
}

void ProjectionCube::FindProjectionPosition(Point *pCurrentPoint, int *pPosition)
{
	int i, j, k = 0;
	float max = 0.0;

	for (int it = 0; it < 3; it++)
	{
		if (abs(pCurrentPoint->P[it]) > max)
		{
			max = abs(pCurrentPoint->P[it]);
			k = it;
		}
	}

	if (pCurrentPoint->P[k] < 0)
		k = k + 3;

	float p_[3];
	RVLMULMX3X3TVECT(RCube[k], pCurrentPoint->P, p_);

	i = (int)(round(cubeHalfSize * p_[0] / p_[2])) + cubeHalfSize;
	j = (int)(round(cubeHalfSize * p_[1] / p_[2])) + cubeHalfSize;

	pPosition[0] = i;
	pPosition[1] = j;
	pPosition[2] = k;

}


void ProjectionCube::ProjectOnCube(Mesh *pMesh, float *R, float *t)
//void ProjectionCube::ProjectOnCube(Mesh *pMesh, float *R, float *t, char *FileName)
{ 	
	RVL_DELETE_ARRAY(transformedPoints.Element);

	//float R0[9];
	//std::memcpy(R0, R, 3 * 3 * sizeof(float));

	transformedPoints.Element = new Point[pMesh->NodeArray.n];
	transformedPoints.n = pMesh->NodeArray.n;

	int noCubeSideRows = 2 * cubeHalfSize + 1;
	int noCubeSideColumns = 2 * cubeHalfSize + 1;

	for (int i = 0; i < 6; i++)
		memset(cubeMatrix[i], 0xff, sizeof(int) * noCubeSideRows * noCubeSideColumns);		//pre-set all cells to -1

	int pPosition[3];	

	//FILE* pCubeFile = NULL;
	//char *viewFileName = RVLCreateFileName(FileName, ".ply", -1, "_cube.txt");
	//pCubeFile = fopen(viewFileName, "w");

	//if (pCubeFile == NULL)
	//{
	//	printf("Cube correspodences file could not be created.");
	//}

	//FILE* pTrsfPtsFile = NULL;
	//char *viewFileName2 = RVLCreateFileName(FileName, ".ply", -1, "_trsfPts.txt");
	//pTrsfPtsFile = fopen(viewFileName2, "w");

	transformedPoints.n = 0;

	for (int pointIndex = 0; pointIndex < pMesh->NodeArray.n; pointIndex++)
	{
		Point *pCurrentPoint = pMesh->NodeArray.Element + pointIndex;

		Point *pTransformedPt = transformedPoints.Element + pointIndex;

		if (pCurrentPoint->bValid == false)
		{
			//fprintf(pTrsfPtsFile, "%d %.5f %.5f %.5f %.5f %.5f %.5f\n", pointIndex, 0, 0, 0, 0, 0, 0);
			RVLNULL3VECTOR(pTransformedPt->P);
			continue;
		}
		else
		{
			float N_lenght = RVLDOTPRODUCT3(pCurrentPoint->N, pCurrentPoint->N);
			if (N_lenght > 0.9)
			{								
				RVLTRANSF3(pCurrentPoint->P, R, t, pTransformedPt->P);
				RVLMULMX3X3VECT(R, pCurrentPoint->N, pTransformedPt->N);

				FindProjectionPosition(pTransformedPt, pPosition);
				cubeMatrix[pPosition[2]][pPosition[0] + pPosition[1] * noCubeSideColumns] = pointIndex;
				//fprintf(pCubeFile, "%d %d %d %d\n", pointIndex, pPosition[2], pPosition[1], pPosition[0]);
				//fprintf(pTrsfPtsFile, "%d %.5f %.5f %.5f %.5f %.5f %.5f\n", pointIndex, pTransformedPt->P[0], pTransformedPt->P[1], pTransformedPt->P[2], pTransformedPt->N[0], pTransformedPt->N[1], pTransformedPt->N[2]);
			}
			else
			{
				//fprintf(pTrsfPtsFile, "%d %.5f %.5f %.5f %.5f %.5f %.5f\n", pointIndex, 0, 0, 0, 0, 0, 0);
				RVLNULL3VECTOR(pTransformedPt->P);
				continue;
			}

		}
			
	}

	transformedPoints.n = pMesh->NodeArray.n;

	//fclose(pCubeFile);
	//fclose(pTrsfPtsFile);

	//delete[] viewFileName;
	//delete[] viewFileName2;
}

void ProjectionCube::ProjectCentralPoints(Array<Point> centralPoints_array, float *R_cameraView, float *t_cameraView)
{
	
	RVL_DELETE_ARRAY(transformedPoints.Element);
	int count_1 = 0;
	int count_2 = 0;

	// Image properties
	int image_width = 320;
	int image_height = 240;

	transformedPoints.Element = new Point[centralPoints_array.n];
	transformedPoints.n = centralPoints_array.n;

	int noCubeSideRows = 2 * cubeHalfSize + 1;
	int noCubeSideColumns = 2 * cubeHalfSize + 1;

	for (int i = 0; i < 6; i++)
		memset(cubeMatrix[i], 0, sizeof(int) * noCubeSideRows * noCubeSideColumns);		//pre-set all cells to zero

	int pPosition[3];

	transformedPoints.n = 0;

	for (int pointIndex = 0; pointIndex < centralPoints_array.n; pointIndex++)
	{
		Point *pCurrentPoint = centralPoints_array.Element + pointIndex;
		Point *pTransformedPt = transformedPoints.Element + pointIndex;
		
		//Transform central point to projection cube 
		RVLTRANSF3(pCurrentPoint->P, R_cameraView, t_cameraView, pTransformedPt->P);
		RVLMULMX3X3VECT(R_cameraView, pCurrentPoint->N, pTransformedPt->N);

		FindProjectionPosition(pTransformedPt, pPosition);

		//ovdje provjeriti u kojem je regionu pPosition i onda spremiti 1 ili 2, umjesto pointIndexa

		// ne valja....ne provjeravam jel original tockica unutar specificciranog regiona, neg transformirana

		if ((pPosition[0] > image_width / 4) && (pPosition[0] < 3 * image_width / 4) && (pPosition[1] > image_height / 4) && (pPosition[1] < 3 * image_height / 4))
		{
			cubeMatrix[pPosition[2]][pPosition[0] + pPosition[1] * noCubeSideColumns] = 2;
			count_2++;
		}
		else
		{
			cubeMatrix[pPosition[2]][pPosition[0] + pPosition[1] * noCubeSideColumns] = 1;
			count_1++;
		}
			
	}
}


void ProjectionCube::CorrespondenceCube()
{

}


void ProjectionCube::Clear()
{
	//funkcija za brisanje inicijalizirane memorije
	//RVL_DELETE_ARRAY(fileName);
	RVL_DELETE_ARRAY(transformedPoints.Element);

	for (int i = 0; i < 6; i++)
		delete[] cubeMatrix[i];
}