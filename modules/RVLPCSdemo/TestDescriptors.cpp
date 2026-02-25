#include <Windows.h>	// for time measurement only!

#include "Platform.h"
#include "RVLCore.h"
#include "RVLPCS.h"
#include "PSGMCommon.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/shot_lrf_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/common/transforms.h>

#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <pcl/correspondence.h>

#include <omp.h>

#include <boost\algorithm\string.hpp>

#include "TestDescriptors.h"


using namespace std;
//typedef pcl::PointXYZRGBA PointType;
//typedef pcl::SHOT1344 DescriptorType;

typedef pcl::PointXYZRGBA PointTypeRGB;
typedef pcl::PointXYZ PointType;
typedef pcl::SHOT352 DescriptorType;
typedef pcl::SHOT1344 DescriptorTypeRGB;
typedef pcl::Normal NormalType;




void GenerateSHOTDescriptors(
	CRVLPCSVS* pVS, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene, 
	float *pKeypoints, 
	float r,
	DESCRIPTOR_INFO *pSHOTI, 
	float &tNormals,
	float &tDescriptors,
	bool bSHOTColor)
{
	LARGE_INTEGER tStart, tEnd;
	LARGE_INTEGER frequency;
	QueryPerformanceFrequency((LARGE_INTEGER *)&frequency);
	
	int nThreads = 3;
	float normal_radiusSearch = 1.0e+2f;
	float descriptor_radiusSearch = r;
	

	//Create an array of pointers to hold the descriptors of each (convex) segment
	float **ppSHOTDescriptors = new float *[pSHOTI->iNoOfFeatures];
	memset(ppSHOTDescriptors, 0, pSHOTI->iNoOfFeatures * sizeof(float));
	pSHOTI->ppDescriptors = ppSHOTDescriptors;
	
	float *pCurrentSHOTDescriptor;

	//Normals for  SHOT descriptor calculations
	pcl::PointCloud<NormalType>::Ptr normals(new pcl::PointCloud<NormalType>());

	if (bSHOTColor)
	{
		//Variables for  SHOT descriptor calculations
		pcl::search::KdTree<PointTypeRGB>::Ptr tree(new pcl::search::KdTree<PointTypeRGB>());
		pcl::PointCloud<PointTypeRGB>::Ptr keypoints(new pcl::PointCloud<PointTypeRGB>(pVS->m_nObjects, 1));
		pcl::PointCloud<DescriptorTypeRGB>::Ptr descriptors(new pcl::PointCloud<DescriptorTypeRGB>());

		//keypoints->height = 1;
		//keypoints->width = pVS->m_nObjects;
		keypoints->is_dense = false;
		//keypoints->points.resize(pVS->m_nObjects);

		//Copy over keypoints
		for (int i = 0; i < pVS->m_nObjects; i++)
		{
			//keypoints->points[i].x = pKeypoints[i * 3 + 0];
			//keypoints->points[i].y = pKeypoints[i * 3 + 1];
			//keypoints->points[i].z = pKeypoints[i * 3 + 2];
			keypoints->at(i).x = pKeypoints[i * 3 + 0];
			keypoints->at(i).y = pKeypoints[i * 3 + 1];
			keypoints->at(i).z = pKeypoints[i * 3 + 2];
		}

		QueryPerformanceCounter((LARGE_INTEGER *)&tStart);
		//Compute Normals
		pcl::NormalEstimationOMP<PointTypeRGB, NormalType> norm_est;
		norm_est.setNumberOfThreads(nThreads);
		norm_est.setInputCloud(scene);
		norm_est.setSearchMethod(tree);
		norm_est.setRadiusSearch(normal_radiusSearch);
		norm_est.compute(*normals);
		QueryPerformanceCounter((LARGE_INTEGER *)&tEnd);
		tNormals = (tEnd.QuadPart - tStart.QuadPart) * 1000.0 / frequency.QuadPart;


		//Compute descriptors
		QueryPerformanceCounter((LARGE_INTEGER *)&tStart);
		pcl::SHOTColorEstimationOMP<PointTypeRGB, NormalType, DescriptorTypeRGB> descr_est;
		descr_est.setNumberOfThreads(nThreads);
		descr_est.setInputCloud(keypoints);
		descr_est.setInputNormals(normals);
		descr_est.setSearchSurface(scene);
		descr_est.setSearchMethod(tree);
		descr_est.setRadiusSearch(descriptor_radiusSearch);
		descr_est.compute(*descriptors);
		QueryPerformanceCounter((LARGE_INTEGER *)&tEnd);
		tDescriptors = (tEnd.QuadPart - tStart.QuadPart) * 1000.0 / frequency.QuadPart;

		//Define SHOT INFO params
		pSHOTI->iDescriptorLength = descriptors->points[0].descriptorSize();

		//Copy over the descriptor values 
		for (int i = 0; i < descriptors->size(); i++)
		{
			pCurrentSHOTDescriptor = new float[pSHOTI->iDescriptorLength];
			ppSHOTDescriptors[i] = pCurrentSHOTDescriptor;

			for (int j = 0; j < descriptors->points[i].descriptorSize(); j++)
			{
				pCurrentSHOTDescriptor[j] = descriptors->points[i].descriptor[j];
			}
		}

		
	}
	else
	{
		//Convert RGB scene to Grayscale
		pcl::PointCloud<pcl::PointXYZ>::Ptr sceneGray(new pcl::PointCloud<pcl::PointXYZ>(scene->width, scene->height));

		sceneGray->is_dense = false;

		for (size_t i = 0; i < scene->points.size(); i++) {
			sceneGray->points[i].x = scene->points[i].x;
			sceneGray->points[i].y = scene->points[i].y;
			sceneGray->points[i].z = scene->points[i].z;
		}


		//Variables for  SHOT descriptor calculations
		pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
		pcl::PointCloud<PointType>::Ptr keypoints(new pcl::PointCloud<PointType>());
		pcl::PointCloud<DescriptorType>::Ptr descriptors(new pcl::PointCloud<DescriptorType>());

		keypoints->height = 1;
		keypoints->width = pVS->m_nObjects;
		keypoints->is_dense = false;
		keypoints->points.resize(pVS->m_nObjects);


		//Copy over keypoints
		for (int i = 0; i < pVS->m_nObjects; i++)
		{
			keypoints->points[i].x = pKeypoints[i * 3 + 0];
			keypoints->points[i].y = pKeypoints[i * 3 + 1];
			keypoints->points[i].z = pKeypoints[i * 3 + 2];
		}

		QueryPerformanceCounter((LARGE_INTEGER *)&tStart);
		//Compute Normals
		pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
		norm_est.setNumberOfThreads(nThreads);
		norm_est.setInputCloud(sceneGray);
		norm_est.setSearchMethod(tree);
		norm_est.setRadiusSearch(normal_radiusSearch);
		norm_est.compute(*normals);
		QueryPerformanceCounter((LARGE_INTEGER *)&tEnd);
		tNormals = (tEnd.QuadPart - tStart.QuadPart) * 1000.0 / frequency.QuadPart;

		
		//Compute descriptors
		QueryPerformanceCounter((LARGE_INTEGER *)&tStart);
		pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
		descr_est.setNumberOfThreads(nThreads);
		descr_est.setInputCloud(keypoints);
		descr_est.setInputNormals(normals);
		descr_est.setSearchSurface(sceneGray);
		descr_est.setSearchMethod(tree);
		descr_est.setRadiusSearch(descriptor_radiusSearch);
		descr_est.compute(*descriptors);
		QueryPerformanceCounter((LARGE_INTEGER *)&tEnd);
		tDescriptors = (tEnd.QuadPart - tStart.QuadPart) * 1000.0 / frequency.QuadPart;

		//Define SHOT INFO params
		pSHOTI->iDescriptorLength = descriptors->points[0].descriptorSize();

		//Copy over the descriptor values 
		for (int i = 0; i < descriptors->size(); i++)
		{
			pCurrentSHOTDescriptor = new float[pSHOTI->iDescriptorLength];
			ppSHOTDescriptors[i] = pCurrentSHOTDescriptor;

			for (int j = 0; j < descriptors->points[i].descriptorSize(); j++)
			{
				pCurrentSHOTDescriptor[j] = descriptors->points[i].descriptor[j];
			}
		}		

	}

	

}


void GenerateSHOTKeypoints(CRVLPCSVS* pVS, float *pKeypoints)
{
	//Triangle list
	CRVL2DRegion2 *pTriangle;

	vector<int> vecNoOfConvexSegmentsPerLabel, vecNoOf3DPtsPerLabel;
	vector<PixelPoint> vecCentroidPerLabel;

	int nearestPos;  //calculate the nearest point to the centroid with depth value


	//Helper variables
	RVL3DPOINT2 *pPoint3D;

	vecNoOfConvexSegmentsPerLabel.resize(pVS->m_nObjects);
	vecCentroidPerLabel.resize(pVS->m_nObjects);
	vecNoOf3DPtsPerLabel.resize(pVS->m_nObjects, 0);


	CRVLMPtrChain *pTriangleList = &(pVS->m_AImage.m_C2DRegion.m_ObjectList);

	pTriangleList->Start();

	while (pTriangleList->m_pNext)
	{
		pTriangle = (CRVL2DRegion2 *)(pTriangleList->GetNext());

		if (pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		if (pTriangle->m_Label < 0 || pTriangle->m_Label >= pVS->m_nObjects)
			continue;


		vecNoOfConvexSegmentsPerLabel[pTriangle->m_Label]++;
		vecNoOf3DPtsPerLabel[pTriangle->m_Label] += pTriangle->m_n3DPts;
		for (int i = 0; i < pTriangle->m_n3DPts; i++)
		{
			vecCentroidPerLabel[pTriangle->m_Label].u += pTriangle->m_pPoint3DArray[i]->u;
			vecCentroidPerLabel[pTriangle->m_Label].v += pTriangle->m_pPoint3DArray[i]->v;
		}
	}


	for (int i = 0; i < pVS->m_nObjects; i++)
	{

		vecCentroidPerLabel[i].u = vecCentroidPerLabel[i].u / vecNoOf3DPtsPerLabel[i];
		vecCentroidPerLabel[i].v = vecCentroidPerLabel[i].v / vecNoOf3DPtsPerLabel[i];
		
		pPoint3D = pVS->m_PSD.m_Point3DMap[vecCentroidPerLabel[i].u + vecCentroidPerLabel[i].v * 320];
		int t = 0, e = 0;

		if (pPoint3D == NULL)
		{
			nearestPos = 5 + 5 * 320; //max udaljenost tocke od centroida u pretrazivanju matrice
			if (vecCentroidPerLabel[i].v > 5 && vecCentroidPerLabel[i].v <= 234)
			{
				for (int k = vecCentroidPerLabel[i].v - 5; k <= vecCentroidPerLabel[i].v + 5; k++)
				{
					if (vecCentroidPerLabel[i].u > 5 && vecCentroidPerLabel[i].u <= 314)
					{
						for (int j = vecCentroidPerLabel[i].u - 5; j <= vecCentroidPerLabel[i].u + 5; j++)
						{
							if ((pVS->m_PSD.m_Point3DMap[j + k * 320] != NULL) && ((j < vecCentroidPerLabel[i].u && k < vecCentroidPerLabel[i].v) || (abs(vecCentroidPerLabel[i].u - j) + abs(vecCentroidPerLabel[i].v - k) < nearestPos)))
							{
								pPoint3D = pVS->m_PSD.m_Point3DMap[j + k * 320];
								nearestPos = abs(vecCentroidPerLabel[i].u - j) + abs(vecCentroidPerLabel[i].v - k);
								t = j;
								e = k;
							}
						}
					}
					else if (vecCentroidPerLabel[i].u <= 5)
					{
						for (int j = 0; j <= vecCentroidPerLabel[i].u + 5; j++)
						{
							if ((pVS->m_PSD.m_Point3DMap[j + k * 320] != NULL) && ((j < vecCentroidPerLabel[i].u && k < vecCentroidPerLabel[i].v) || (abs(vecCentroidPerLabel[i].u - j) + abs(vecCentroidPerLabel[i].v - k) < nearestPos)))
							{
								pPoint3D = pVS->m_PSD.m_Point3DMap[j + k * 320];
								nearestPos = abs(vecCentroidPerLabel[i].u - j) + abs(vecCentroidPerLabel[i].v - k);
								t = j;
								e = k;
							}
						}
					}
					else
					{
						for (int j = vecCentroidPerLabel[i].u - 5; j < 320; k++)
						{
							if ((pVS->m_PSD.m_Point3DMap[j + k * 320] != NULL) && ((j < vecCentroidPerLabel[i].u && k < vecCentroidPerLabel[i].v) || (abs(vecCentroidPerLabel[i].u - j) + abs(vecCentroidPerLabel[i].v - k) < nearestPos)))
							{
								pPoint3D = pVS->m_PSD.m_Point3DMap[j + k * 320];
								nearestPos = abs(vecCentroidPerLabel[i].u - j) + abs(vecCentroidPerLabel[i].v - k);
								t = j;
								e = k;
							}
						}
					}
				}
			}
			else if (vecCentroidPerLabel[i].v <= 5)
			{
				for (int k = 0; k <= vecCentroidPerLabel[i].v + 5; k++)
				{
					if (vecCentroidPerLabel[i].u > 5 && vecCentroidPerLabel[i].u <= 314)
					{
						for (int j = vecCentroidPerLabel[i].u - 5; j <= vecCentroidPerLabel[i].u + 5; j++)
						{
							if ((pVS->m_PSD.m_Point3DMap[j + k * 320] != NULL) && ((j < vecCentroidPerLabel[i].u && k < vecCentroidPerLabel[i].v) || (abs(vecCentroidPerLabel[i].u - j) + abs(vecCentroidPerLabel[i].v - k) < nearestPos)))
							{
								pPoint3D = pVS->m_PSD.m_Point3DMap[j + k * 320];
								nearestPos = abs(vecCentroidPerLabel[i].u - j) + abs(vecCentroidPerLabel[i].v - k);
								t = j;
								e = k;
							}
						}
					}
					else if (vecCentroidPerLabel[i].u <= 5)
					{
						for (int j = 0; j <= vecCentroidPerLabel[i].u + 5; j++)
						{
							if ((pVS->m_PSD.m_Point3DMap[j + k * 320] != NULL) && ((j < vecCentroidPerLabel[i].u && k < vecCentroidPerLabel[i].v) || (abs(vecCentroidPerLabel[i].u - j) + abs(vecCentroidPerLabel[i].v - k) < nearestPos)))
							{
								pPoint3D = pVS->m_PSD.m_Point3DMap[j + k * 320];
								nearestPos = abs(vecCentroidPerLabel[i].u - j) + abs(vecCentroidPerLabel[i].v - k);
								t = j;
								e = k;
							}
						}
					}
					else
					{
						for (int j = vecCentroidPerLabel[i].u - 5; j < 320; j++)
						{
							if ((pVS->m_PSD.m_Point3DMap[j + k * 320] != NULL) && ((j < vecCentroidPerLabel[i].u && k < vecCentroidPerLabel[i].v) || (abs(vecCentroidPerLabel[i].u - j) + abs(vecCentroidPerLabel[i].v - k) < nearestPos)))
							{
								pPoint3D = pVS->m_PSD.m_Point3DMap[j + k * 320];
								nearestPos = abs(vecCentroidPerLabel[i].u - j) + abs(vecCentroidPerLabel[i].v - k);
								t = j;
								e = k;
							}
						}
					}
				}
			}
			else if (vecCentroidPerLabel[i].v>234)
			{
				for (int k = vecCentroidPerLabel[i].v - 5; k <= 240; k++)
				{
					if (vecCentroidPerLabel[i].u > 5 && vecCentroidPerLabel[i].u <= 314)
					{
						for (int j = vecCentroidPerLabel[i].u - 5; j <= vecCentroidPerLabel[i].u + 5; j++)
						{
							if ((pVS->m_PSD.m_Point3DMap[j + k * 320] != NULL) && ((j < vecCentroidPerLabel[i].u && k < vecCentroidPerLabel[i].v) || (abs(vecCentroidPerLabel[i].u - j) + abs(vecCentroidPerLabel[i].v - k) < nearestPos)))
							{
								pPoint3D = pVS->m_PSD.m_Point3DMap[j + k * 320];
								nearestPos = abs(vecCentroidPerLabel[i].u - j) + abs(vecCentroidPerLabel[i].v - k);
								t = j;
								e = k;
							}
						}
					}
					else if (vecCentroidPerLabel[i].u <= 5)
					{
						for (int j = 0; j <= vecCentroidPerLabel[i].u + 5; j++)
						{
							if ((pVS->m_PSD.m_Point3DMap[j + k * 320] != NULL) && ((j < vecCentroidPerLabel[i].u && k < vecCentroidPerLabel[i].v) || (abs(vecCentroidPerLabel[i].u - j) + abs(vecCentroidPerLabel[i].v - k) < nearestPos)))
							{
								pPoint3D = pVS->m_PSD.m_Point3DMap[j + k * 320];
								nearestPos = abs(vecCentroidPerLabel[i].u - j) + abs(vecCentroidPerLabel[i].v - k);
								t = j;
								e = k;
							}
						}
					}
					else
					{
						for (int j = vecCentroidPerLabel[i].u - 5; j < 320; j++)
						{
							if ((pVS->m_PSD.m_Point3DMap[j + k * 320] != NULL) && ((j < vecCentroidPerLabel[i].u && k < vecCentroidPerLabel[i].v) || (abs(vecCentroidPerLabel[i].u - j) + abs(vecCentroidPerLabel[i].v - k) < nearestPos)))
							{
								pPoint3D = pVS->m_PSD.m_Point3DMap[j + k * 320];
								nearestPos = abs(vecCentroidPerLabel[i].u - j) + abs(vecCentroidPerLabel[i].v - k);
								t = j;
								e = k;
							}
						}
					}
				}
			}
			if (pPoint3D == NULL)
			{
				cout << "U= " << vecCentroidPerLabel[i].u << " V= " << vecCentroidPerLabel[i].v << " at place: " << i << endl;
				continue;  //osiguranje da i nakon 11x11 matrice ne postoji tocka sa x, y, i z koordinatama
			}
			else
			{
				pKeypoints[i * 3 + 0] = pPoint3D->x;
				pKeypoints[i * 3 + 1] = pPoint3D->y;
				pKeypoints[i * 3 + 2] = pPoint3D->z;

			}
		}
		else
		{
			pKeypoints[i * 3 + 0] = pPoint3D->x;
			pKeypoints[i * 3 + 1] = pPoint3D->y;
			pKeypoints[i * 3 + 2] = pPoint3D->z;
		}

	}

	
}

void GenerateCTIDescriptors(CRVLPCSVS *pVS, DESCRIPTOR_INFO *pCTI, Array2D<float> *pDescriptorTemplate)
{
	//Create an array of pointers to hold the vertices of each convex segment
	double **ppConvexSegmentVertices = new double *[pCTI->iNoOfFeatures];
	memset(ppConvexSegmentVertices, 0, pCTI->iNoOfFeatures * sizeof(double));

	//Create a counter array
	int *arrNTrianglesInConvexSegment = new int[pCTI->iNoOfFeatures];
	memset(arrNTrianglesInConvexSegment, 0, pCTI->iNoOfFeatures * sizeof(int));

	//Create an array of pointers to hold the descriptors of each convex segment
	float **ppConvexSegmentDescriptors = new float *[pCTI->iNoOfFeatures];
	memset(ppConvexSegmentDescriptors, 0, pCTI->iNoOfFeatures * sizeof(float));

	//Create an array of pointers to hold the normalized descriptors of each convex segment
	float **ppConvexSegmentNormalizedDescriptors = new float *[pCTI->iNoOfFeatures];
	memset(ppConvexSegmentNormalizedDescriptors, 0, pCTI->iNoOfFeatures * sizeof(float));
	pCTI->ppDescriptors = ppConvexSegmentNormalizedDescriptors;




	
	//1. Get all vertices grouped by convex segments
	double *pCurrentConvexSegmentVerticeSet, *pNewConvexSegmentVerticeSet;
	CRVL2DRegion2 *pTriangle;
	RVLMESH_LINK *pLink;
	RVL3DPOINT2 *p3DPt0, *p3DPt1, *p3DPt2;
	int currNoPoints;


	//Go through all triangles and add the vertices of all triangles with the same label to the same set 
	CRVLMPtrChain *pTriangleList = &(pVS->m_AImage.m_C2DRegion.m_ObjectList);

	pTriangleList->Start();
	while (pTriangleList->m_pNext)
	{
		pTriangle = (CRVL2DRegion2 *)(pTriangleList->GetNext());

		if ((pTriangle->m_Label > pCTI->iNoOfFeatures) || (pTriangle->m_Label < 0))
			continue;

		if (pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;


		if (pTriangle->m_Flags & RVLOBJ2_FLAG_VISITED)
			continue;


		//Set visited flag
		pTriangle->m_Flags |= RVLOBJ2_FLAG_VISITED;


		//Get new points/vertices
		pLink = (RVLMESH_LINK *)(pTriangle->m_PtArray);

		p3DPt0 = pVS->m_PSD.m_Point3DMap[pLink->iPix0];

		p3DPt1 = pVS->m_PSD.m_Point3DMap[pLink->pNext->pOpposite->iPix0];

		p3DPt2 = pVS->m_PSD.m_Point3DMap[pLink->pOpposite->iPix0];

		//Get current number of points
		currNoPoints = arrNTrianglesInConvexSegment[pTriangle->m_Label];

		pNewConvexSegmentVerticeSet = new double[3 * (currNoPoints + 3)];

		if (currNoPoints > 0)
		{
			//Get current set of vertices
			pCurrentConvexSegmentVerticeSet = ppConvexSegmentVertices[pTriangle->m_Label];

			//Copy over the previous set of vertices
			for (int i = 0; i < currNoPoints; i++)
			{
				pNewConvexSegmentVerticeSet[3 * i + 0] = pCurrentConvexSegmentVerticeSet[3 * i + 0];
				pNewConvexSegmentVerticeSet[3 * i + 1] = pCurrentConvexSegmentVerticeSet[3 * i + 1];
				pNewConvexSegmentVerticeSet[3 * i + 2] = pCurrentConvexSegmentVerticeSet[3 * i + 2];
			}
			//Delete previous set
			delete[] pCurrentConvexSegmentVerticeSet;
		}

		//Add new vertices
		pNewConvexSegmentVerticeSet[3 * currNoPoints + 0] = p3DPt0->x;
		pNewConvexSegmentVerticeSet[3 * currNoPoints + 1] = p3DPt0->y;
		pNewConvexSegmentVerticeSet[3 * currNoPoints + 2] = p3DPt0->z;

		pNewConvexSegmentVerticeSet[3 * (currNoPoints + 1) + 0] = p3DPt1->x;
		pNewConvexSegmentVerticeSet[3 * (currNoPoints + 1) + 1] = p3DPt1->y;
		pNewConvexSegmentVerticeSet[3 * (currNoPoints + 1) + 2] = p3DPt1->z;

		pNewConvexSegmentVerticeSet[3 * (currNoPoints + 2) + 0] = p3DPt2->x;
		pNewConvexSegmentVerticeSet[3 * (currNoPoints + 2) + 1] = p3DPt2->y;
		pNewConvexSegmentVerticeSet[3 * (currNoPoints + 2) + 2] = p3DPt2->z;

		//Store new set
		ppConvexSegmentVertices[pTriangle->m_Label] = pNewConvexSegmentVerticeSet;



		//Update corresponding counter
		arrNTrianglesInConvexSegment[pTriangle->m_Label] = currNoPoints + 3;
	}


	// Reset visited flag
	pTriangleList->Start();
	while (pTriangleList->m_pNext)
	{
		pTriangle = (CRVL2DRegion2 *)(pTriangleList->GetNext());

		pTriangle->m_Flags &= ~RVLOBJ2_FLAG_VISITED;
	}



	//2. Create CTI (normalize) descriptors for each convex segment



	//Get alpha for normalization procedure later on
	double a0 = 0;
	double aTa[9]; // 3 x 3
	double aTaInv[9];

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			a0 = 0.0;

			for (int k = 0; k < pDescriptorTemplate->h; k++)
				a0 += pDescriptorTemplate->Element[k * 3 + i] * pDescriptorTemplate->Element[k * 3 + j];

			aTa[i * 3 + j] = a0;
		}
	}
	InverseMatrix3(aTaInv, aTa);




	//Create CTI descriptors for each convex segment
	float *pCurrentConvexSegmentDescriptor;
	float *pCurrentConvexSegmentNormalizedDescriptor;

	float *pT = new float[3];
	float *pT2 = new float[3];

	for (int labelNo = 0; labelNo < pCTI->iNoOfFeatures; labelNo++)
	{
		//if (labelNo == 19)
		//	int debug = 0;

		//Check if vertices exist
		currNoPoints = arrNTrianglesInConvexSegment[labelNo];

		if (currNoPoints > 0)
		{
			//Get current set of vertices
			pCurrentConvexSegmentVerticeSet = ppConvexSegmentVertices[labelNo];

			pCurrentConvexSegmentDescriptor = new float[pDescriptorTemplate->h];
			pCurrentConvexSegmentNormalizedDescriptor = new float[pDescriptorTemplate->h];

			ppConvexSegmentDescriptors[labelNo] = pCurrentConvexSegmentDescriptor;
			ppConvexSegmentNormalizedDescriptors[labelNo] = pCurrentConvexSegmentNormalizedDescriptor;

			//CTI descriptor
			float _val = 0;
			for (int i = 0; i < pDescriptorTemplate->h; i++)
			{
				pCurrentConvexSegmentDescriptor[i] = -1E10;

				for (int ipt = 0; ipt < currNoPoints; ipt++)
				{
					_val = pCurrentConvexSegmentVerticeSet[ipt * 3 + 0] * pDescriptorTemplate->Element[i * 3 + 0] +
						pCurrentConvexSegmentVerticeSet[ipt * 3 + 1] * pDescriptorTemplate->Element[i * 3 + 1] +
						pCurrentConvexSegmentVerticeSet[ipt * 3 + 2] * pDescriptorTemplate->Element[i * 3 + 2];

					if (_val > pCurrentConvexSegmentDescriptor[i])
					{
						pCurrentConvexSegmentDescriptor[i] = _val;
					}
				}

			}

			//Normalized CTI descriptor
			//Reset pT
			pT[0] = 0;
			pT[1] = 0;
			pT[2] = 0;

			for (int j = 0; j < 3; j++)
			{
				for (int i = 0; i < pDescriptorTemplate->h; i++)
				{
					pT[j] += (pDescriptorTemplate->Element[i * 3 + j] * pCurrentConvexSegmentDescriptor[i]);
				}
			}

			pT2[0] = 0;
			pT2[1] = 0;
			pT2[2] = 0;
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					pT2[i] += aTaInv[i * 3 + j] * pT[j];
				}
			}

			float *pAt = new float[pDescriptorTemplate->h];
			for (int i = 0; i < pDescriptorTemplate->h; i++)
			{
				pAt[i] = pDescriptorTemplate->Element[i * 3 + 0] * pT2[0]
					+ pDescriptorTemplate->Element[i * 3 + 1] * pT2[1]
					+ pDescriptorTemplate->Element[i * 3 + 2] * pT2[2];


				pCurrentConvexSegmentNormalizedDescriptor[i] = pCurrentConvexSegmentDescriptor[i] - pAt[i];
			}


			delete[] pAt;


		}
	}

	delete[] pT;
	delete[] pT2;

	for (int i = 0; i < pCTI->iNoOfFeatures; i++)
	{
		delete[] ppConvexSegmentDescriptors[i];
		delete[] ppConvexSegmentVertices[i];
	}
	delete[] ppConvexSegmentDescriptors;
	delete[] ppConvexSegmentVertices;
	delete[] arrNTrianglesInConvexSegment;
	
}

void SaveSegmentsAndDescriptorsToFile(CRVLPCSVS *pVS, DESCRIPTOR_INFO *pDescInfo, int iPrevONISample, char *Extension, bool bAddCTIColor)
{

	char *DescriptorFileName = RVLKinectCreateONISampleFileName(pVS->m_Kinect.m_ONIFileName, iPrevONISample, Extension);

	SaveSegmentsAndDescriptors(&(pVS->m_AImage.m_C2DRegion.m_ObjectList), pVS->m_nObjects, pDescInfo, RVLOBJ2_FLAG_MARKED, DescriptorFileName, bAddCTIColor);

	delete[] DescriptorFileName;

}

void SaveSegmentsAndDescriptors(
	CRVLMPtrChain *pTriangleList,
	int nSegments,
	DESCRIPTOR_INFO *pDescInfo,
	DWORD Flags,
	char *FileName, 
	bool bAddCTIColor)
{
	int descriptorLength = pDescInfo->iDescriptorLength;
	float **ppDescriptors = pDescInfo->ppDescriptors;

	bool *bSegmentSelected = new bool[nSegments];

	memset(bSegmentSelected, 0, nSegments * sizeof(bool));

	bool bSelectedSegments = false;

	CRVL2DRegion2 *pTriangle;

	pTriangleList->Start();

	while (pTriangleList->m_pNext)
	{
		pTriangle = (CRVL2DRegion2 *)(pTriangleList->GetNext());

		if (pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		if (pTriangle->m_Label < 0 || pTriangle->m_Label >= nSegments)
			continue;

		if (pTriangle->m_Flags & Flags)
		{
			bSelectedSegments = true;

			bSegmentSelected[pTriangle->m_Label] = true;
		}
	}

	if (bSelectedSegments)
	{
		FILE *fp = fopen(FileName, "w");

		for (int iSegment = 0; iSegment < nSegments; iSegment++)
		{
			fprintf(fp, "%d\t%d", iSegment, bSegmentSelected[iSegment]);
			for (int i = 0; i < descriptorLength; i++)
			{
				fprintf(fp, "\t%f", ppDescriptors[iSegment][i]);
			}
			if (bAddCTIColor) //Add color info at the end of the decsriptor
			{
				//fprintf(fp, "\t%f\t%f", (float)(pDescInfo->pDominantColor[2 * iSegment + 0]), (float)(pDescInfo->pDominantColor[2 * iSegment + 1]));
				fprintf(fp, "\t%d\t%d", pDescInfo->pDominantColor[2 * iSegment + 0], pDescInfo->pDominantColor[2 * iSegment + 1]);
			}
			fprintf(fp, "\n");
		}
		fclose(fp);
	}

	delete[] bSegmentSelected;
}



void ConvexTemplate(Array2D<float> &A)
{
	A.w = 3;
	A.h = 66;
	A.Element = new float[A.w * A.h];


	float h = 0.25f * PI;
	float q = 0.5f * h;
	float sh = sin(h);
	float ch = cos(h);
	float sq = sin(q);
	float cq = cos(q);

	float *NT = new float[3 * 13];

	float *N;

	N = NT;
	//RVLSET3VECTOR(N, 0.0f, 0.0f, 1.0f);
	N[0] = 0.0f;
	N[1] = 0.0f;
	N[2] = 1.0f;
	N = NT + 3;
	//RVLSET3VECTOR(N, 0.0f, -ch, ch);
	N[0] = 0.0f;
	N[1] = -ch;
	N[2] = ch;
	N = NT + 2 * 3;
	//RVLSET3VECTOR(N, ch, 0.0f, ch);
	N[0] = ch;
	N[1] = 0.0f;
	N[2] = ch;
	N = NT + 11 * 3;
	//RVLSET3VECTOR(N, 0.0f, ch, ch);
	N[0] = 0.0f;
	N[1] = ch;
	N[2] = ch;
	N = NT + 12 * 3;
	//RVLSET3VECTOR(N, -ch, 0.0f, ch);
	N[0] = -ch;
	N[1] = 0.0f;
	N[2] = ch;

	int templ[] = {
		3, 0, 1,
		4, 0, 2,
		5, 1, 2,
		6, 0, 11,
		7, 0, 12,
		8, 2, 11,
		9, 1, 12,
		10, 11, 12 };

	int i;
	float *N_, *N__;
	float fTmp;

	for (i = 0; i < 8; i++)
	{
		N = NT + 3 * templ[3 * i];
		N_ = NT + 3 * templ[3 * i + 1];
		N__ = NT + 3 * templ[3 * i + 2];
		RVLSUM3VECTORS(N_, N__, N);
		RVLNORM3(N, fTmp);
	}

	float R[] = {
		0.0f, 0.0f, -1.0f,
		1.0f, 0.0f, 0.0f,
		0.0f, -1.0f, 0.0f };

	float R_[9];

	RVLMXMUL3X3(R, R, R_);

	int j;
	int i_;
	//RECOG::PSGM_::Plane *pPlane;

	for (i = 0; i < 6; i++)
	{
		for (j = 0; j < 11; j++)
		{
			N = A.Element + 3 * (11 * i + j);

			N_ = NT + 3 * j;

			i_ = i % 3;

			if (i_ == 0)
			{
				RVLCOPY3VECTOR(N_, N);
			}
			else if (i_ == 1)
			{
				RVLMULMX3X3VECT(R, N_, N)
			}
			else
			{
				RVLMULMX3X3VECT(R_, N_, N)
			}

			if (i >= 3)
			{
				RVLNEGVECT3(N, N);
			}

		}
	}


	//!!!!!!!!!
	//Get only single view normals
	//Determine the elements of ak < 0

	float *pCurrentSingleView, *pNewSingleView;

	int cntr = 0;
	//Flag to indicate that previous data exists
	int bStart = 0;

	for (i = 0; i < 66; i++)
	{
		if (A.Element[i * 3 + 2] < 0)
		{
			//increase counter;
			cntr = cntr + 1;

			//Create new View
			pNewSingleView = new float[3 * cntr];

			if (bStart == 1)
			{
				//Copy from previous 
				for (j = 0; j < cntr - 1; j++)
				{
					pNewSingleView[j * 3 + 0] = pCurrentSingleView[j * 3 + 0];
					pNewSingleView[j * 3 + 1] = pCurrentSingleView[j * 3 + 1];
					pNewSingleView[j * 3 + 2] = pCurrentSingleView[j * 3 + 2];
				}
				delete[] pCurrentSingleView;
			}


			//Add current values
			pNewSingleView[(cntr - 1) * 3 + 0] = A.Element[i * 3 + 0];
			pNewSingleView[(cntr - 1) * 3 + 1] = A.Element[i * 3 + 1];
			pNewSingleView[(cntr - 1) * 3 + 2] = A.Element[i * 3 + 2];

			//Set to mark that previous data exists
			bStart = 1;
			pCurrentSingleView = pNewSingleView;
		}
	}
	A.Element = pCurrentSingleView;
	A.h = cntr; // Store descriptor length


	delete[] NT;
}