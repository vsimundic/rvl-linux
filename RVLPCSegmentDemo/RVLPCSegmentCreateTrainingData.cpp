// RVLPCSegmentDemo.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
#include "RVLVTK.h"
#include <vtkPolyLine.h>
#include "RVLCore2.h"
#include "Util.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
#include "SurfelGraph.h"
#include "PlanarSurfelDetector.h"
#include <pcl/common/common.h>
#include <pcl/PolygonMesh.h>
#include "PCLTools.h"
#include "RGBDCamera.h"
#include "PCLMeshBuilder.h"
//#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>
#include <algorithm>
#include <unordered_set>
#include "ObjectGraph.h"

#define RVLPCSEGMENT_DEMO_FLAG_SAVE_PLY			0x00000001

using namespace RVL;

#include "RVLPCSegmentCreateTrainingData.h"

void CreateParamList(
	CRVLParameterList *pParamList,
	CRVLMem *pMem,
	char **pMeshFileName,
	DWORD &flags,
	bool &bSegmentToObjects,
	bool &bObjectAggregationLevel2,
	char **pSVMClassifierParamsFileName,
	char **pSequenceFileName,
	char **pSegmentationResultsFileName);

#ifdef RVLSURFEL_IMAGE_ADJACENCY
//Returns surfels Label ID with most object support
int DetPrimaryGTObj(Surfel *pSurfel, cv::Mat labGTImg, int noObj)
{
	int objIdx = -1;	//default value
	//Generate histogram of object (pixel) support
	int *objHist = new int[noObj];
	memset(objHist, 0, noObj * sizeof(int));
	RVL::QLIST::Index2 *pt;
	int x = 0, y = 0;
	pt = pSurfel->PtList.pFirst;
	for (int i = 0; i < pSurfel->size; i++)
	{
		y = floor(pt->Idx / 640.0);
		x = floor(pt->Idx - 640.0 * y);
		objHist[labGTImg.at<cv::Vec3b>(y, x)[0]]++;
		pt = pt->pNext;
	}
	//find max support
	int max = 0;
	for (int i = 0; i < noObj; i++)
	{
		if (objHist[i] > max)
		{
			max = objHist[i];
			objIdx = i;
		}
	}
	delete [] objHist;
	return objIdx;
}

//Return shortest distance on the image between some pixel and the pixels on the boundary
float GetShortestDistanceToBoundary(int x, int y, Array<MeshEdgePtr *> &BoundaryArray)
{
	float distance = 0.0, tempdistance = 0.0;
	//run through edges
	MeshEdgePtr *pCurrEdge;
	int iBoundary, iPointEdge;
	int iPt, xx, yy, xm, ym;
	for (iPointEdge = 0; iPointEdge < BoundaryArray.n; iPointEdge++)
	{
		pCurrEdge = BoundaryArray.Element[iPointEdge];

		iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pCurrEdge);	//point on the boundary
		yy = floor(iPt / 640.0);
		xx = floor(iPt - 640.0 * yy);
		xm = xx - x;
		ym = yy - y;
		tempdistance = sqrt((xm * xm + ym * ym));
		if (tempdistance < distance)
			distance = tempdistance;
	}
	return distance;
}

//Draw boundary on the opencv image
void DrawSurfelBoundary(Array<MeshEdgePtr *> &BoundaryArray, cv::Mat img)
{
	//run through edges
	MeshEdgePtr *pCurrEdge;
	int iPointEdge;
	int iPt, x, y;
	for (iPointEdge = 0; iPointEdge < BoundaryArray.n; iPointEdge++)
	{
		pCurrEdge = BoundaryArray.Element[iPointEdge];

		iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pCurrEdge);
		y = floor(iPt / 640.0);
		x = floor(iPt - 640.0 * y);

		img.at<cv::Vec3b>(y, x)[0] = 0;
		img.at<cv::Vec3b>(y, x)[1] = 255;
		img.at<cv::Vec3b>(y, x)[2] = 0;
	}
}

//Check if surfel is valid or the segmentation is poor (surfel across different objects)
bool GetSurfelGTValidity(Surfel *pSurfel, cv::Mat labGTImg, float thr)
{
	//find largest boundary (most probable outer-boundary)
	int boundary = 0;
	int boundarySize = 0;
	if (pSurfel->BoundaryArray.n > 1)
	{
		for (int b = 0; b < pSurfel->BoundaryArray.n; b++)
		{
			if ((pSurfel->BoundaryArray.Element + b)->n > boundarySize)
			{
				boundarySize = (pSurfel->BoundaryArray.Element + b)->n;
				boundary = b;
			}
		}
	}

	//Pass trough all surfel pixels
	RVL::QLIST::Index2 *pt;
	int x = 0, y = 0;
	pt = pSurfel->PtList.pFirst;
	float shDistance = 0.0;
	for (int i = 0; i < pSurfel->size; i++)
	{
		y = floor(pt->Idx / 640.0);
		x = floor(pt->Idx - 640.0 * y);
		//check if pixel label is different from surfel label
		if (labGTImg.at<cv::Vec3b>(y, x)[0] != pSurfel->ObjectID)
		{
			//Find shortest distance to boundary
			shDistance = GetShortestDistanceToBoundary(x, y, pSurfel->BoundaryArray.Element[boundary]);
			if (shDistance > thr)
				return false;
		}
		pt = pt->pNext;
	}
	return true;
}

//Fill a list o neighbouring surfel pointers that belong to different object
void FindSurfelNeighbours(std::vector<Surfel*> &nList, Surfel *pSurfel, SurfelGraph *surfels, int thr)
{
	//find largest boundary to point on the boundary
	int boundary = 0;
	int boundarySize = 0;
	if (pSurfel->BoundaryArray.n > 1)
	{
		for (int b = 0; b < pSurfel->BoundaryArray.n; b++)
		{
			if ((pSurfel->BoundaryArray.Element + b)->n > boundarySize)
			{
				boundarySize = (pSurfel->BoundaryArray.Element + b)->n;
				boundary = b;
			}
		}
	}

	//run through edges
	Array<MeshEdgePtr *> BoundaryArray = pSurfel->BoundaryArray.Element[boundary];
	MeshEdgePtr *pCurrEdge;
	Surfel *pOtherSurfel;
	int iBoundary, iPointEdge;
	int iPt, x, y;
	for (iPointEdge = 0; iPointEdge < BoundaryArray.n; iPointEdge++)
	{
		pCurrEdge = BoundaryArray.Element[iPointEdge];

		iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pCurrEdge);
		y = floor(iPt / 640.0);
		x = floor(iPt - 640.0 * y);
		//Running through point neighbourhood
		for (int yy = y - thr; yy < y + thr; yy++)
		{
			if ((yy < 0) || (yy >= 480))
				continue;
			for (int xx = x - thr; xx < x + thr; xx++)
			{
				if ((xx < 0) || (xx >= 640))
					continue;
				pOtherSurfel = &surfels->NodeArray.Element[surfels->surfelMap[yy * 640 + xx]];
				if ((pOtherSurfel->size < 640*480) && (pOtherSurfel->size != 0) && (pOtherSurfel->size != 1) && (pOtherSurfel->ObjectID > 0) /*&& (pOtherSurfel->ObjectID != pSurfel->ObjectID)*/ && (std::find(nList.begin(), nList.end(), pOtherSurfel) == nList.end()))
				{
					nList.push_back(pOtherSurfel);
				}
			}
		}
	}
}

//Sets a list of neighbouring surfels
void SetSurfelImgAdjacency(Surfel *pSurfel, SurfelGraph *surfels, Mesh *mesh, int thr)
{
	//find largest boundary (most probable outer boundary)
	int boundary = 0;
	int boundarySize = 0;
	if (pSurfel->BoundaryArray.n > 1)
	{
		for (int b = 0; b < pSurfel->BoundaryArray.n; b++)
		{
			if (pSurfel->BoundaryArray.Element[b].n > boundarySize)
			{
				boundarySize = pSurfel->BoundaryArray.Element[b].n;
				boundary = b;
			}
		}
	}
	else
		boundarySize = pSurfel->BoundaryArray.Element[boundary].n;

	//run through edges
	Array<MeshEdgePtr *> BoundaryArray = pSurfel->BoundaryArray.Element[boundary];
	MeshEdgePtr *pCurrEdge;
	Surfel *pOtherSurfel;
	SurfelAdjecencyDescriptors *desc;
	int iBoundary, iPointEdge;
	int iPt, iPt2, x, y;
	std::vector<Surfel*>::iterator surfIt;
	double tempDist;
	int iOtherSurfel;
	int i;
	std::unordered_set<int> visitedSurfels;	//visited surfels (for common boundary lenght) (indices of imgAdjacency list)
	for (iPointEdge = 0; iPointEdge < BoundaryArray.n; iPointEdge++)
	{
		pCurrEdge = BoundaryArray.Element[iPointEdge];

		iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pCurrEdge);
		y = floor(iPt / 640.0);
		x = floor(iPt - 640.0 * y);
		visitedSurfels.clear();
		//Running through point neighbourhood
		for (int yy = y - thr; yy < y + thr; yy++)
		{
			if ((yy < 0) || (yy >= 480))
				continue;
			for (int xx = x - thr; xx < x + thr; xx++)
			{
				if ((xx < 0) || (xx >= 640))
					continue;
				iPt2 = yy * 640 + xx;
				iOtherSurfel = surfels->surfelMap[iPt2];
				if ((iOtherSurfel < 0) || (iOtherSurfel >= surfels->NodeArray.n))
					continue;
				pOtherSurfel = &surfels->NodeArray.Element[iOtherSurfel];	//surfel owner of the pixel
				surfIt = std::find(pSurfel->imgAdjacency.begin(), pSurfel->imgAdjacency.end(), pOtherSurfel);	//find if that surfel is already on the list
				if ((pOtherSurfel->size < 640 * 480) && (pOtherSurfel->size != 0) && (pOtherSurfel->size != 1) && (pOtherSurfel->ObjectID >= 0) && (pOtherSurfel != pSurfel)/*&& (pOtherSurfel->ObjectID != pSurfel->ObjectID)*/ && (surfIt == pSurfel->imgAdjacency.end()))
				{
					pSurfel->imgAdjacency.push_back(pOtherSurfel);	//push surfel pointer on the list
					//calculate min dist
					//preallocate the adjacency descriptor for future use
					desc = new SurfelAdjecencyDescriptors;
					desc->minDist = sqrt((mesh->NodeArray.Element[iPt].P[0] - mesh->NodeArray.Element[iPt2].P[0]) * (mesh->NodeArray.Element[iPt].P[0] - mesh->NodeArray.Element[iPt2].P[0]) + (mesh->NodeArray.Element[iPt].P[1] - mesh->NodeArray.Element[iPt2].P[1]) * (mesh->NodeArray.Element[iPt].P[1] - mesh->NodeArray.Element[iPt2].P[1]) + (mesh->NodeArray.Element[iPt].P[2] - mesh->NodeArray.Element[iPt2].P[2]) * (mesh->NodeArray.Element[iPt].P[2] - mesh->NodeArray.Element[iPt2].P[2]));
					desc->cupyDescriptor[0] = 0.0;
					desc->cupyDescriptor[1] = 0.0;
					desc->cupyDescriptor[2] = 0.0;
					desc->cupyDescriptor[3] = 0.0;
					desc->commonBoundaryLength = 0;
					pSurfel->imgAdjacencyDescriptors.push_back(desc);	//push descriptor on the list

					//push to other surfel
					pOtherSurfel->imgAdjacency.push_back(pSurfel);
					pOtherSurfel->imgAdjacencyDescriptors.push_back(desc);
					
					visitedSurfels.insert(pSurfel->imgAdjacency.size() - 1);	//Visited surfel on the list
				}
				else if (surfIt != pSurfel->imgAdjacency.end())	//If it is on the list, find and update min distance
				{
					desc = pSurfel->imgAdjacencyDescriptors.at(surfIt - pSurfel->imgAdjacency.begin());	//get related descriptor
					tempDist = sqrt((mesh->NodeArray.Element[iPt].P[0] - mesh->NodeArray.Element[iPt2].P[0]) * (mesh->NodeArray.Element[iPt].P[0] - mesh->NodeArray.Element[iPt2].P[0]) + (mesh->NodeArray.Element[iPt].P[1] - mesh->NodeArray.Element[iPt2].P[1]) * (mesh->NodeArray.Element[iPt].P[1] - mesh->NodeArray.Element[iPt2].P[1]) + (mesh->NodeArray.Element[iPt].P[2] - mesh->NodeArray.Element[iPt2].P[2]) * (mesh->NodeArray.Element[iPt].P[2] - mesh->NodeArray.Element[iPt2].P[2]));
					if (tempDist < desc->minDist)	//update if the new one is smaller
						desc->minDist = tempDist;
					visitedSurfels.insert(surfIt - pSurfel->imgAdjacency.begin());	//Visited surfel on the list
				}
			}
		}	//Running through point neighbourhood

		//Updating Common boundary lenght
		for (const int& s : visitedSurfels)
		{
			desc = pSurfel->imgAdjacencyDescriptors.at(s);	//get related descriptor
			desc->commonBoundaryLength++;
		}
	}	// for every boundary point
}

//Return surfel's centroind on the image. Useful for 2D visualization
void GetSurfelImgCentroid(int &x, int &y, Surfel *pSurfel)
{
	x = 0;
	y = 0;
	RVL::QLIST::Index2 *pt;
	pt = pSurfel->PtList.pFirst;
	int xx, yy;
	for (int i = 0; i < pSurfel->size; i++)
	{
		yy = floor(pt->Idx / 640.0);
		xx = floor(pt->Idx - 640.0 * yy);
		x += xx;
		y += yy;
		pt = pt->pNext;
	}
	x /= pSurfel->size;
	y /= pSurfel->size;
}

//Adds vtk line between two points as an independent actor with specified color to a specified renderer
void AddVTKLine(float *P1, float *P2, vtkSmartPointer<vtkRenderer> renderer, double *color)
{
	//Adding points
	vtkSmartPointer<vtkPoints> points =	vtkSmartPointer<vtkPoints>::New();
	points->InsertNextPoint(P1[0], P1[1], P1[2]);
	points->InsertNextPoint(P2[0], P2[1], P2[2]);
	//Adding line
	vtkSmartPointer<vtkPolyLine> polyLine =	vtkSmartPointer<vtkPolyLine>::New();
	polyLine->GetPointIds()->SetNumberOfIds(2);
	polyLine->GetPointIds()->SetId(0, 0);
	polyLine->GetPointIds()->SetId(1, 1);
	vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
	cells->InsertNextCell(polyLine);
	//Polydata
	vtkSmartPointer<vtkPolyData> polyData =	vtkSmartPointer<vtkPolyData>::New();
	polyData->SetPoints(points);
	polyData->SetLines(cells);
	//Mapper & actor
	// Setup actor and mapper
	vtkSmartPointer<vtkPolyDataMapper> mapper =	vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(polyData);
	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(color);
	//Add actor
	renderer->AddActor(actor);
}

//Generates a colored opencv image based on the label image
cv::Mat GenColoredGTImg(cv::Mat labImg)
{
	cv::Mat coloredLab(480, 640, CV_8UC3);
	//Get min/max value (num of colors)
	double minLab, maxLab;
	cv::minMaxLoc(labImg, &minLab, &maxLab);
	//Generate colors and colorize labels
	unsigned char *labColors = new unsigned char[3 * (int)(maxLab + 1)];
	labColors[0] = 0;	//Black
	labColors[1] = 0;
	labColors[2] = 0;
	for (int i = 1; i <= maxLab; i++)
	{
		labColors[3 * i] = rand() % 255;
		labColors[3 * i + 1] = rand() % 255;
		labColors[3 * i + 2] = rand() % 255;
	}
	//coloring image
	for (int y = 0; y < 480; y++)
	{
		for (int x = 0; x < 640; x++)
		{
			coloredLab.at<cv::Vec3b>(y, x)[0] = labColors[3 * labImg.at<cv::Vec3b>(y, x)[0]];
			coloredLab.at<cv::Vec3b>(y, x)[1] = labColors[3 * labImg.at<cv::Vec3b>(y, x)[0] + 1];
			coloredLab.at<cv::Vec3b>(y, x)[2] = labColors[3 * labImg.at<cv::Vec3b>(y, x)[0] + 2];
		}
	}
	//return image
	return coloredLab;
}

//Generate a colored opencv image based on the surfel segmentation
cv::Mat GenColoredSegmentationImg(SurfelGraph *surfels)
{
	//Segmentation colored label image
	cv::Mat coloredSegLab(480, 640, CV_8UC3, cv::Scalar::all(0));
	Surfel *pCurrSurfel = surfels->NodeArray.Element;
	RVL::QLIST::Index2 *pt;
	unsigned char labSegColor[3];
	int x = 0, y = 0;
	for (int i = 0; i < surfels->NodeArray.n; pCurrSurfel++, i++)
	{
		if ((pCurrSurfel->size == 1) || (pCurrSurfel->size == 0) || pCurrSurfel->bEdge)
			continue;
		//Generate surfel color
		labSegColor[0] = rand() % 255;
		labSegColor[1] = rand() % 255;
		labSegColor[2] = rand() % 255;
		pt = pCurrSurfel->PtList.pFirst;
		//Set pixel colors
		for (int k = 0; k < pCurrSurfel->size; k++)
		{
			y = floor(pt->Idx / 640.0);
			x = floor(pt->Idx - 640.0 * y);
			coloredSegLab.at<cv::Vec3b>(y, x)[0] = labSegColor[0];
			coloredSegLab.at<cv::Vec3b>(y, x)[1] = labSegColor[1];
			coloredSegLab.at<cv::Vec3b>(y, x)[2] = labSegColor[2];
			pt = pt->pNext;
		}
	}
	//return image
	return coloredSegLab;
}

//Draw adjacency lines on preexisting image. Green lines for surfels belonging to same object and red lines for different objects
void DrawSurfelImgAdjacencyOpenCV(cv::Mat img, SurfelGraph *surfels, bool checkbackground = true)
{
	//Connection
	int x = 0, y = 0, x0 = 0, y0 = 0;
	Surfel *pCurrSurfel = surfels->NodeArray.Element;
	Surfel *pOtherSurfel;
	for (int i = 0; i < surfels->NodeArray.n; pCurrSurfel++, i++)
	{
		if ((pCurrSurfel->ObjectID == -1) || (checkbackground && ((pCurrSurfel->ObjectID == 255) || (pCurrSurfel->ObjectID == 0))) || (pCurrSurfel->size == 1) || (pCurrSurfel->size == 0) || pCurrSurfel->bEdge)
			continue;

		GetSurfelImgCentroid(x, y, pCurrSurfel);
		for (int a = 0; a < pCurrSurfel->imgAdjacency.size(); a++)
		{
			pOtherSurfel = pCurrSurfel->imgAdjacency.at(a);
			GetSurfelImgCentroid(x0, y0, pOtherSurfel);
			if (pCurrSurfel->ObjectID != pOtherSurfel->ObjectID)
				cv::line(img, cv::Point(x, y), cv::Point(x0, y0), cv::Scalar(0, 0, 255)/*::all(255)*/);
			else
				cv::line(img, cv::Point(x, y), cv::Point(x0, y0), cv::Scalar(0, 255, 0));
		}
	}
}

//Render adjacency lines on VTK renderer. Green lines for surfels belonging to same object and red lines for different objects
void RenderSurfelImgAdjacencyVTK(vtkSmartPointer<vtkRenderer> renderer, SurfelGraph *surfels, bool checkbackground = true)
{
	Surfel *pCurrSurfel = surfels->NodeArray.Element;
	Surfel *pOtherSurfel;
	double colorRed[3] = { 255, 0, 0 };
	double colorGreen[3] = { 0, 255, 0 };
	for (int i = 0; i < surfels->NodeArray.n; pCurrSurfel++, i++)
	{
		if ((pCurrSurfel->ObjectID == -1) || (checkbackground && ((pCurrSurfel->ObjectID == 255) || (pCurrSurfel->ObjectID == 0))) || (pCurrSurfel->size == 1) || (pCurrSurfel->size == 0) || pCurrSurfel->bEdge)
			continue;

		for (int a = 0; a < pCurrSurfel->imgAdjacency.size(); a++)
		{
			pOtherSurfel = pCurrSurfel->imgAdjacency.at(a);
			if (pCurrSurfel->ObjectID != pOtherSurfel->ObjectID)
				AddVTKLine(pCurrSurfel->P, pOtherSurfel->P, renderer, colorRed);
			else
				AddVTKLine(pCurrSurfel->P, pOtherSurfel->P, renderer, colorGreen);			
		}
	}
}

//Preprocess label image
void PreprocessGTLab(cv::Mat GTLabImg, cv::Mat GTDisparityImg)
{
	//Set background pixels (depth but no label) to 255
	for (int y = 0; y < 480; y++)
	{
		for (int x = 0; x < 640; x++)
		{
			//labeling surroundings that have valid depth value
			if ((GTLabImg.at<cv::Vec3b>(y, x)[0] == 0) && GTDisparityImg.at<unsigned short>(y, x) > 0)
			{
				GTLabImg.at<cv::Vec3b>(y, x)[0] = 255;
				GTLabImg.at<cv::Vec3b>(y, x)[1] = 255;
				GTLabImg.at<cv::Vec3b>(y, x)[2] = 255;
			}
		}
	}
}

////Calculate adjacency descriptors
//void DetermineImgAdjDescriptors(Surfel *pSurfel, Mesh *mesh)
//{
//	//Calculate Cupy adjacency descriptor
//	//find largest boundary (most probable outer boundary)
//	int boundary = 0;
//	int boundarySize = 0;
//	if (pSurfel->BoundaryArray.n > 1)
//	{
//		for (int b = 0; b < pSurfel->BoundaryArray.n; b++)
//		{
//			if (pSurfel->BoundaryArray.Element[b].n > boundarySize)
//			{
//				boundarySize = pSurfel->BoundaryArray.Element[b].n;
//				boundary = b;
//			}
//		}
//	}
//	else
//		boundarySize = pSurfel->BoundaryArray.Element[boundary].n;
//
//	//run through neighbours
//	Array<MeshEdgePtr *> BoundaryArray;
//	MeshEdgePtr *pCurrEdge;
//	Surfel *pOtherSurfel;
//	SurfelAdjecencyDescriptors *desc;
//	int iBoundary, iPointEdge;
//	int iPt, x, y;
//	double a[4];
//	double tempN[3];
//	double dOffset;
//	for (int i = 0; i < pSurfel->imgAdjacency.size(); i++)
//	{
//		//Get other surfel
//		pOtherSurfel = pSurfel->imgAdjacency.at(i);
//		desc = pSurfel->imgAdjacencyDescriptors.at(i);
//		if ((desc->cupyDescriptor[0] + desc->cupyDescriptor[1] + desc->cupyDescriptor[2] + desc->cupyDescriptor[3]) != 0.0)
//			continue; //this adjacancy descriptor has already been set, probably by other surfel
//		//get other boundary
//		int boundaryOther = 0;
//		int boundarySizeOther = 0;
//		if (pOtherSurfel->BoundaryArray.n > 1)
//		{
//			for (int b = 0; b < pOtherSurfel->BoundaryArray.n; b++)
//			{
//				if (pOtherSurfel->BoundaryArray.Element[b].n > boundarySizeOther)
//				{
//					boundarySizeOther = pOtherSurfel->BoundaryArray.Element[b].n;
//					boundaryOther = b;
//				}
//			}
//		}
//		else
//			boundarySizeOther = pOtherSurfel->BoundaryArray.Element[boundaryOther].n;
//
//		memset(a, 0, 4 * sizeof(double));
//		tempN[0] = pOtherSurfel->N[0] - pSurfel->N[0];
//		tempN[1] = pOtherSurfel->N[1] - pSurfel->N[1];
//		tempN[2] = pOtherSurfel->N[2] - pSurfel->N[2];
//		dOffset = pOtherSurfel->d - pSurfel->d;
//		//calculate ... something ... with current surfel
//		BoundaryArray = pSurfel->BoundaryArray.Element[boundary];
//		for (iPointEdge = 0; iPointEdge < BoundaryArray.n; iPointEdge++)
//		{
//			pCurrEdge = BoundaryArray.Element[iPointEdge];
//
//			iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pCurrEdge);
//
//			if ((tempN[0] * mesh->NodeArray.Element[iPt].P[0] + tempN[1] * mesh->NodeArray.Element[iPt].P[1] + tempN[2] * mesh->NodeArray.Element[iPt].P[2] - dOffset) <= 0.0)
//				a[0]++;
//			
//		}
//		if (boundarySize > 0)
//			a[0] /= (double)boundarySize;
//		else
//			a[0] = 0.0f;
//		a[1] = 1.0 - a[0];
//
//		tempN[0] = pSurfel->N[0] - pOtherSurfel->N[0];
//		tempN[1] = pSurfel->N[1] - pOtherSurfel->N[1];
//		tempN[2] = pSurfel->N[2] - pOtherSurfel->N[2];
//		dOffset = pSurfel->d - pOtherSurfel->d;
//		//calculate ... something ... with current other surfel
//		BoundaryArray = pOtherSurfel->BoundaryArray.Element[boundaryOther];
//		for (iPointEdge = 0; iPointEdge < BoundaryArray.n; iPointEdge++)
//		{
//			pCurrEdge = BoundaryArray.Element[iPointEdge];
//
//			iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pCurrEdge);
//
//			if ((tempN[0] * mesh->NodeArray.Element[iPt].P[0] + tempN[1] * mesh->NodeArray.Element[iPt].P[1] + tempN[2] * mesh->NodeArray.Element[iPt].P[2] - dOffset) <= 0.0)
//				a[2]++;
//
//		}
//		if (boundarySizeOther > 0)
//			a[2] /= (double)boundarySizeOther;
//		else
//			a[2] = 0.0f;
//		a[3] = 1.0 - a[2];
//
//		int p, q;
//		double tempm = 0;
//		//argmax_l(max_i(a_i_l))
//		for (int pp = 0; pp < 4; pp++)
//		{
//			if (a[pp] > tempm)
//			{
//				p = pp;
//				tempm = a[pp];
//			}
//		}
//		p = (p % 2 == 0) ? 1 : 2;
//		//argmax_i(a_i_p)
//		q = (a[p - 1] > a[2 + p - 1]) ? 1 : 2;
//
//		//get and update descriptor
//		desc->cupyDescriptor[0] = (3 - 2 * p) * acos(pSurfel->N[0] * pOtherSurfel->N[0] + pSurfel->N[1] * pOtherSurfel->N[1] + pSurfel->N[2] * pOtherSurfel->N[2]);	//(3 - 2*p)*acos(n_i*n_j)
//		desc->cupyDescriptor[1] = a[(q - 1) * 2 + (p - 1)];
//		desc->cupyDescriptor[2] = a[((3 - q) - 1) * 2 + (p - 1)];
//		desc->cupyDescriptor[3] = desc->minDist;
//	}	
//	
//}

void ComputeRelationFeatures(
	SurfelGraph *pSurfels, 
	Mesh *pMesh)
{
	pSurfels->ImageAdjacency(pMesh);

	Surfel *pSurfel = pSurfels->NodeArray.Element;

	for (int i = 0; i < pSurfels->NodeArray.n; pSurfel++, i++)
	{
		if (pSurfel->size <= 1)
			continue;

		pSurfels->DetermineImgAdjDescriptors(pSurfel, pMesh);
	}
}

////Generate scene segmenation file
//void GenerateSSF(SurfelGraph *surfels, std::string filename, int minSurfelSize, bool checkbackground)
//{
//	std::stringstream ss;
//	//SceneSegFile object
//	SceneSegFile::SceneSegFile* ssf = new SceneSegFile::SceneSegFile("Scene");
//	std::shared_ptr<SceneSegFile::SegFileElement> surfel;
//
//	Surfel *pCurrSurfel = surfels->NodeArray.Element;
//	Surfel *pOtherSurfel;
//	//for surfel
//	for (int i = 0; i < surfels->NodeArray.n; pCurrSurfel++, i++)
//	{
//		//if ((pCurrSurfel->ObjectID == -1) || (checkbackground && ((pCurrSurfel->ObjectID == 255) || (pCurrSurfel->ObjectID == 0))) || (pCurrSurfel->size == 1) || (pCurrSurfel->size == 0) || pCurrSurfel->bEdge || pCurrSurfel->size < minSurfelSize)
//		if ((checkbackground && ((pCurrSurfel->ObjectID == 255) || (pCurrSurfel->ObjectID == 0))) || (pCurrSurfel->size <= 1) || pCurrSurfel->bEdge)
//			continue;
//
//		//Create element
//		ss.clear();
//		ss.str("");
//		ss << "Surfel_" << i;
//		surfel = std::make_shared<SceneSegFile::SegFileElement>(i, ss.str());
//		ssf->AddElement(surfel);
//
//		//Add surfel features
//		//Centroid
//		surfel->features.AddFeature(SceneSegFile::FeaturesList::Centroid, SceneSegFile::FeaturesDictionary::dictionary.at(SceneSegFile::FeaturesList::Centroid), "float");
//		surfel->features.CopyFeatureData<float>(SceneSegFile::FeaturesList::Centroid, pCurrSurfel->P, 3);
//		//GT object ID
//		surfel->features.AddFeature(SceneSegFile::FeaturesList::GTObjectID, SceneSegFile::FeaturesDictionary::dictionary.at(SceneSegFile::FeaturesList::GTObjectID), "int");
//		surfel->features.CopyFeatureData<int>(SceneSegFile::FeaturesList::GTObjectID, &pCurrSurfel->ObjectID, 1);
//		//Pixel affiliation
//		int *pixelIndices = new int[pCurrSurfel->size];
//		RVL::QLIST::Index2 *pt;
//		pt = pCurrSurfel->PtList.pFirst;
//		for (int i = 0; i < pCurrSurfel->size; i++)
//		{
//			pixelIndices[i] = pt->Idx;
//			pt = pt->pNext;
//		}
//		surfel->features.AddFeature(SceneSegFile::FeaturesList::PixelAffiliation, SceneSegFile::FeaturesDictionary::dictionary.at(SceneSegFile::FeaturesList::PixelAffiliation), "int");
//		surfel->features.SetFeatureData<int>(SceneSegFile::FeaturesList::PixelAffiliation, pixelIndices, pCurrSurfel->size);
//		//GTObjHistogram
//		int *GTObjHist = new int[pCurrSurfel->GTObjHist.size()];
//		for (int i = 0; i < pCurrSurfel->GTObjHist.size(); i++)
//			GTObjHist[i] = pCurrSurfel->GTObjHist[i];
//		surfel->features.AddFeature(SceneSegFile::FeaturesList::GTObjHistogram, SceneSegFile::FeaturesDictionary::dictionary.at(SceneSegFile::FeaturesList::GTObjHistogram), "int");
//		surfel->features.SetFeatureData<int>(SceneSegFile::FeaturesList::GTObjHistogram, GTObjHist, pCurrSurfel->GTObjHist.size());
//		//Add feature groups
//		//Adjacency group
//		surfel->AddFeatureGroup(SceneSegFile::FeatureGroupsList::AdjacencyFeatureGroup, SceneSegFile::FeatureGroupsDictionary::dictionary.at(SceneSegFile::FeatureGroupsList::AdjacencyFeatureGroup));
//		std::shared_ptr<SceneSegFile::FeatureGroup> adjFeatureGroup = surfel->featureGroups.at(SceneSegFile::FeatureGroupsList::AdjacencyFeatureGroup);
//
//		//Adjacency group's set
//		for (int i = 0; i < pCurrSurfel->imgAdjacency.size(); i++)
//		{
//			pOtherSurfel = pCurrSurfel->imgAdjacency.at(i);
//			//feature group's feature set
//			adjFeatureGroup->AddFeatureSet(pOtherSurfel - surfels->NodeArray.Element, SceneSegFile::FeatureSetsDictionary::dictionary.at(SceneSegFile::FeatureSetsList::AdjacencyNode), true);
//			std::shared_ptr<SceneSegFile::FeatureSet> adjFeatureSet = adjFeatureGroup->featureSets.at(pOtherSurfel - surfels->NodeArray.Element);
//			//Add features
//			//Same GT object
//			adjFeatureSet->AddFeature(SceneSegFile::FeaturesList::SameGTObject, SceneSegFile::FeaturesDictionary::dictionary.at(SceneSegFile::FeaturesList::SameGTObject), "bool");
//			bool sameGTObj = pCurrSurfel->ObjectID == pOtherSurfel->ObjectID ? true : false;
//			adjFeatureSet->SetFeatureData<bool>(SceneSegFile::FeaturesList::SameGTObject, &sameGTObj);
//			//Cupy feature vector
//			adjFeatureSet->AddFeature(SceneSegFile::FeaturesList::CupysFeature, SceneSegFile::FeaturesDictionary::dictionary.at(SceneSegFile::FeaturesList::CupysFeature), "double");
//			adjFeatureSet->CopyFeatureData<double>(SceneSegFile::FeaturesList::CupysFeature, pCurrSurfel->imgAdjacencyDescriptors.at(i)->cupyDescriptor, 4);
//			//CommonBoundaryLenght
//			adjFeatureSet->AddFeature(SceneSegFile::FeaturesList::CommonBoundaryLength, SceneSegFile::FeaturesDictionary::dictionary.at(SceneSegFile::FeaturesList::CommonBoundaryLength), "int");
//			adjFeatureSet->CopyFeatureData<int>(SceneSegFile::FeaturesList::CommonBoundaryLength, &pCurrSurfel->imgAdjacencyDescriptors.at(i)->commonBoundaryLength, 1);
//		}
//	}
//
//	ssf->Save(filename);
//	delete ssf;
//}

void RunSeg2Bench(bool save)
{
	//SURFEL::ObjectGraph gra;
	//gra.CreateFromSSF("C:/Users/Damir/Documents/Project_benchmark_data/OSD-0.2/combined/learn17.ssf");
	
	//Cupec
	// Create memory storage.

	CRVLMem mem0;	// permanent memory

	mem0.Create(1000000);

	CRVLMem mem;	// cycle memory

	mem.Create(100000000);

	// Read parameters from a configuration file.

	char *MeshFileName = NULL;
	char *SVMClassifierParamsFileName = NULL;
	char *SequenceFileName = NULL;
	char *SegmentationResultsFileName = NULL;

	DWORD flags = 0x00000000;
	bool bSegmentToObjects = false;
	bool bObjectAggregationLevel2 = false;

	CRVLParameterList ParamList;

	//CreateParamList(&ParamList, &mem0, &MeshFileName, flags, bSegmentToObjects, bObjectAggregationLevel2);
	CreateParamList(&ParamList, &mem0, &MeshFileName, flags, bSegmentToObjects, bObjectAggregationLevel2, &SVMClassifierParamsFileName, &SequenceFileName, &SegmentationResultsFileName);


	ParamList.LoadParams("RVLPCSegmentDemo.cfg");

	// Read mesh from file.

	PCLMeshBuilder meshBuilder;

	meshBuilder.CreateParamList(&mem0);

	meshBuilder.ParamList.LoadParams("RVLPCSegmentDemo.cfg");

	int w = 640;
	int h = 480;

	Mesh mesh;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC(new pcl::PointCloud<pcl::PointXYZRGBA>(w, h));
	pcl::PolygonMesh PCLMesh;

	printf("Creating mesh from %s:\n", MeshFileName);

	if (meshBuilder.Load(MeshFileName, &mesh, PC, PCLMesh, (flags & RVLPCSEGMENT_DEMO_FLAG_SAVE_PLY) != 0))
		printf("Mesh created.\n");
	else
		printf("ERROR: Mesh can't be created!\n");

	// Segment mesh to surfels.

	SurfelGraph surfels;

	surfels.Init(&mesh);

	PlanarSurfelDetector detector;

	detector.CreateParamList(&mem0);

	detector.ParamList.LoadParams("RVLPCSegmentDemo.cfg");

	detector.Init(&mesh, &surfels, &mem);

	detector.pTimer = new CRVLTimer;

	printf("Segmentation to surfels...");

	double StartTime = detector.pTimer->GetTime();

	detector.Segment(&mesh, &surfels);

	double ExecTime = detector.pTimer->GetTime() - StartTime;

	printf("completed.\n");
	printf("No. of surfels = %d\n", surfels.NodeArray.n);
	printf("Total segmentation time = %lf s\n", ExecTime);

	// Display mesh.

	unsigned char SelectionColor[3];

	SelectionColor[0] = 0;
	SelectionColor[1] = 255;
	SelectionColor[2] = 0;

	surfels.NodeColors(SelectionColor);

	// free memory
	delete detector.pTimer;
	//delete[] MeshFileName;

	//Filko
	//parameters
	int validityDist = 5;	//max distance from border
	int neighbourhoodDist = 6;	//from -neighbourhoodDist to +neighbourhoodDist
	bool checkbackground = false;
	//Segmentation analysis
	//filenames
	std::string labelImgFileName(MeshFileName);
	labelImgFileName.erase(labelImgFileName.find_last_of("."));
	std::string depthImgFileName = labelImgFileName + "d.png";
	std::string ssfFileName = labelImgFileName + ".ssf";
	labelImgFileName += "a.png";
	delete[] MeshFileName;
	////TEST SSF LOAD
	//SceneSegFile::SceneSegFile* ssf = new SceneSegFile::SceneSegFile("test");
	//ssf->Load(ssfFileName);
	//
	//Load label image
	cv::Mat GTlabImg = cv::imread(labelImgFileName);
	cv::Mat GTdepthImg = cv::imread(depthImgFileName, cv::ImreadModes::IMREAD_ANYDEPTH);
	////Preprocess GT label image (such as labeling background)
	//PreprocessGTLab(GTlabImg, GTdepthImg);
	//Get label min/max value
	double minLab, maxLab;
	cv::minMaxLoc(GTlabImg, &minLab, &maxLab);

	//Detect primary GT object for ALL surfels
	Surfel *pCurrSurfel = surfels.NodeArray.Element;
	std::cout << "Detecting primary GT object!" << std::endl;
	for (int i = 0; i < surfels.NodeArray.n; pCurrSurfel++, i++)
	{
		pCurrSurfel->ObjectID = -1;
		if ((pCurrSurfel->size == 1) || (pCurrSurfel->size == 0) || pCurrSurfel->bEdge || pCurrSurfel->size < detector.minSurfelSize)
			continue;
		/*pCurrSurfel->ObjectID = DetPrimaryGTObj(pCurrSurfel, GTlabImg, 256);*/ //256 objects because background has label of 255
		surfels.SetPrimaryGTObj(pCurrSurfel, GTlabImg, maxLab + 1); //maxLab + 1 because the last GT object label has to be maxLab and not maxLab - 1
	}

	//Adjacency and its descriptors
	pCurrSurfel = surfels.NodeArray.Element;
	std::cout << "Finding image adjacency and determining descriptors!" << std::endl;
	for (int i = 0; i < surfels.NodeArray.n; pCurrSurfel++, i++)
	{
		if ((pCurrSurfel->ObjectID == -1) || (checkbackground && ((pCurrSurfel->ObjectID == 255) || (pCurrSurfel->ObjectID == 0))) || (pCurrSurfel->size == 1) || (pCurrSurfel->size == 0) || pCurrSurfel->bEdge || pCurrSurfel->size < detector.minSurfelSize)
			continue;
		//Check surfel validity
		if (GetSurfelGTValidity(pCurrSurfel, GTlabImg, validityDist))
		{
			//Set Surfel naighbours
			SetSurfelImgAdjacency(pCurrSurfel, &surfels, &mesh, neighbourhoodDist);
			//Calculate descriptors
			surfels.DetermineImgAdjDescriptors(pCurrSurfel, &mesh);
		}
	}

	//Same as SSF (SceneSegFile)
	if (save)
	{
		std::cout << "Saving SSF!" << std::endl;
		surfels.GenerateSSF(ssfFileName, detector.minSurfelSize, checkbackground);
		std::cout << "Saved!" << std::endl;
	}

	//Visualization
	//OpenCV visualization
	//GT label image
	cv::imshow("Colored GT label", GenColoredGTImg(GTlabImg));
	//Segmentation label image
	cv::Mat segLabImg = GenColoredSegmentationImg(&surfels);
	//Draw Img adjacency links wrt GT
	DrawSurfelImgAdjacencyOpenCV(segLabImg, &surfels, checkbackground);
	cv::imshow("Colored Segmentation label", segLabImg);
	cv::waitKey(1);
	//VTK visualization
	Visualizer visualizer;
	visualizer.Create();
	surfels.InitDisplay(&visualizer, &mesh, &detector);
	surfels.Display(&visualizer, &mesh);
	detector.DisplaySoftEdges(&visualizer, &mesh, &surfels, SelectionColor);
	//Render Img adjacency links wrt GT
	RenderSurfelImgAdjacencyVTK(visualizer.renderer, &surfels, checkbackground);
	visualizer.Run();
}

////Generate a colored opencv image based on surfel data from SSF
//cv::Mat GenColoredSurfelImgFromSSF(std::shared_ptr<SceneSegFile::SceneSegFile> ssf)
//{
//	std::shared_ptr<SceneSegFile::SegFileElement> currSSFElement;
//	std::shared_ptr<SceneSegFile::FeatureTypeInt> pixAff;
//
//	cv::Mat coloredSegLab(480, 640, CV_8UC3, cv::Scalar::all(0));
//
//	unsigned char labSegColor[3];
//	int x = 0, y = 0;
//
//	for (int i = 0; i < ssf->elements.size(); i++)
//	{
//		currSSFElement = ssf->elements.at(i);
//
//		pixAff = std::dynamic_pointer_cast<SceneSegFile::FeatureTypeInt>(currSSFElement->features.features.at(SceneSegFile::FeaturesList::PixelAffiliation));
//
//		//Generate surfel color
//		labSegColor[0] = rand() % 255;
//		labSegColor[1] = rand() % 255;
//		labSegColor[2] = rand() % 255;
//
//		//Set pixel colors
//		for (int k = 0; k < pixAff->size; k++)
//		{
//			y = floor(pixAff->data[k] / 640.0);
//			x = floor(pixAff->data[k] - 640.0 * y);
//			coloredSegLab.at<cv::Vec3b>(y, x)[0] = labSegColor[0];
//			coloredSegLab.at<cv::Vec3b>(y, x)[1] = labSegColor[1];
//			coloredSegLab.at<cv::Vec3b>(y, x)[2] = labSegColor[2];
//		}
//
//	}
//	//return image
//	return coloredSegLab;
//}

//Generate a colored opencv image based on surfel data from SSF
cv::Mat GenColoredSegmentationImgFromObjectGraph_SSF(SURFEL::ObjectGraph* objects)
{
	std::shared_ptr<SceneSegFile::SceneSegFile> ssf = objects->ssf;

	std::shared_ptr<SceneSegFile::SegFileElement> currSSFElement;
	std::shared_ptr<SceneSegFile::FeatureTypeInt> pixAff;

	cv::Mat coloredSegLab(480, 640, CV_8UC3, cv::Scalar::all(0));

	GRAPH::AggregateNode<SURFEL::AgEdge> *pObject;
	QLIST::Index *piElement;

	unsigned char labSegColor[3];
	int x = 0, y = 0;

	for (int iObject = 0; iObject < objects->NodeArray.n; iObject++)
	{
		//Generate surfel color
		labSegColor[0] = rand() % 255;
		labSegColor[1] = rand() % 255;
		labSegColor[2] = rand() % 255;

		pObject = objects->NodeArray.Element + iObject;

		piElement = pObject->elementList.pFirst;

		while (piElement)
		{
			currSSFElement = ssf->elements.at(piElement->Idx);

			if (piElement->Idx == 2)
				int debug = 0;

			pixAff = std::dynamic_pointer_cast<SceneSegFile::FeatureTypeInt>(currSSFElement->features.features.at(SceneSegFile::FeaturesList::PixelAffiliation));

			//Set pixel colors
			for (int k = 0; k < pixAff->size; k++)
			{
				y = floor(pixAff->data[k] / 640.0);
				x = floor(pixAff->data[k] - 640.0 * y);
				coloredSegLab.at<cv::Vec3b>(y, x)[0] = labSegColor[0];
				coloredSegLab.at<cv::Vec3b>(y, x)[1] = labSegColor[1];
				coloredSegLab.at<cv::Vec3b>(y, x)[2] = labSegColor[2];
			}

			piElement = piElement->pNext;
		}

	}
	//return image
	return coloredSegLab;
}
#endif

////Generate a colored opencv image based on object's surfel data 
//cv::Mat GenColoredSegmentationImgFromObjectGraph(SURFEL::ObjectGraph* objects)
//{
//	cv::Mat coloredSegLab(480, 640, CV_8UC3, cv::Scalar::all(0));
//
//	GRAPH::AggregateNode<SURFEL::AgEdge> *pObject;
//	QLIST::Index *piElement;
//	Surfel *pSurfel;
//	unsigned char labSegColor[3];
//	int x = 0, y = 0;
//	RVL::QLIST::Index2 *pt;
//	srand(time(NULL));
//	for (int iObject = 0; iObject < objects->NodeArray.n; iObject++)
//	{
//		//Generate surfel color
//		labSegColor[0] = rand() % 255;
//		labSegColor[1] = rand() % 255;
//		labSegColor[2] = rand() % 255;
//
//		pObject = objects->NodeArray.Element + iObject;
//
//		piElement = pObject->elementList.pFirst;
//
//		while (piElement)
//		{
//			pSurfel = objects->pSurfels->NodeArray.Element + piElement->Idx;
//			//check 
//			if (!((pSurfel->size <= 1) || pSurfel->bEdge))
//			{
//				pt = pSurfel->PtList.pFirst;
//				//Set pixel colors
//				for (int k = 0; k < pSurfel->size; k++)
//				{
//					y = floor(pt->Idx / 640.0);
//					x = floor(pt->Idx - 640.0 * y);
//					coloredSegLab.at<cv::Vec3b>(y, x)[0] = labSegColor[0];
//					coloredSegLab.at<cv::Vec3b>(y, x)[1] = labSegColor[1];
//					coloredSegLab.at<cv::Vec3b>(y, x)[2] = labSegColor[2];
//					pt = pt->pNext;
//				}
//			}
//			piElement = piElement->pNext;
//		}
//
//	}
//	//return image
//	return coloredSegLab;
//}

//void Seg2Bench()
//{
//	// Create memory storage.
//
//	CRVLMem mem0;	// permanent memory
//
//	mem0.Create(1000000);
//
//	CRVLMem mem;	// cycle memory
//
//	mem.Create(100000000);
//
//	// Read parameters from a configuration file.
//
//	char *MeshFileName = NULL;
//
//	DWORD flags = 0x00000000;
//
//	CRVLParameterList ParamList;
//
//	CreateParamList(&ParamList, &mem0, &MeshFileName, flags);
//
//	ParamList.LoadParams("RVLPCSegmentDemo.cfg");
//
//	// Read mesh from file.
//
//	PCLMeshBuilder meshBuilder;
//
//	meshBuilder.CreateParamList(&mem0);
//
//	meshBuilder.ParamList.LoadParams("RVLPCSegmentDemo.cfg");
//
//	int w = 640;
//	int h = 480;
//
//	Mesh mesh;
//	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC(new pcl::PointCloud<pcl::PointXYZRGBA>(w, h));
//	pcl::PolygonMesh PCLMesh;
//
//	printf("Creating mesh from %s:\n", MeshFileName);
//
//	if (mesh.Load(MeshFileName, &meshBuilder, PC, PCLMesh, (flags & RVLPCSEGMENT_DEMO_FLAG_SAVE_PLY) != 0))
//		printf("Mesh created.\n");
//	else
//		printf("ERROR: Mesh can't be created!\n");
//
//	// Segment mesh to surfels.
//
//	SurfelGraph surfels;
//
//	surfels.Init(&mesh);
//
//	PlanarSurfelDetector detector;
//
//	detector.CreateParamList(&mem0);
//
//	detector.ParamList.LoadParams("RVLPCSegmentDemo.cfg");
//
//	detector.Init(&mesh, &surfels, &mem);
//
//	detector.pTimer = new CRVLTimer;
//
//	printf("Segmentation to surfels...");
//
//	double StartTime = detector.pTimer->GetTime();
//
//	detector.Segment(&mesh, &surfels);
//
//	double ExecTime = detector.pTimer->GetTime() - StartTime;
//
//	printf("completed.\n");
//	printf("No. of surfels = %d\n", surfels.NodeArray.n);
//	printf("Total segmentation time = %lf s\n", ExecTime);
//
//	// Display mesh.
//
//	unsigned char SelectionColor[3];
//
//	SelectionColor[0] = 0;
//	SelectionColor[1] = 255;
//	SelectionColor[2] = 0;
//
//	surfels.NodeColors(SelectionColor);
//
//	Visualizer visualizer;
//
//	visualizer.Create();
//	surfels.InitDisplay(&visualizer, &mesh, &detector);
//	surfels.Display(&visualizer, &mesh);
//	detector.DisplaySoftEdges(&visualizer, &mesh, &surfels, SelectionColor);
//	//visualizer.Run();
//
//	// free memory
//
//	delete detector.pTimer;
//	delete[] MeshFileName;
//
//	//Segmentation analysis
//	//Load label image
//	cv::Mat labImg = cv::imread("C:/Users/Damir/Documents/Project_benchmark_data/OSD-0.2/OSD-0.2-depth/annotation/learn0.png");
//	cv::Mat disparityImg = cv::imread("C:/Users/Damir/Documents/Project_benchmark_data/OSD-0.2/OSD-0.2-depth/disparity/learn0.png", cv::ImreadModes::IMREAD_ANYDEPTH);
//	//Get min/max value (num of colors)
//	double minLab, maxLab;
//	cv::minMaxLoc(labImg, &minLab, &maxLab);
//	//Generate colors and colorize labels
//	unsigned char *labColors = new unsigned char[3 * (int)(maxLab + 1)];
//	labColors[0] = 0;	//Black
//	labColors[1] = 0;
//	labColors[2] = 0;
//	for (int i = 1; i <= maxLab; i++)
//	{
//		labColors[3 * i] = rand() % 255;
//		labColors[3 * i + 1] = rand() % 255;
//		labColors[3 * i + 2] = rand() % 255;
//	}
//	cv::Mat coloredLab(480, 640, CV_8UC3);
//	//color
//	for (int y = 0; y < 480; y++)
//	{
//		for (int x = 0; x < 640; x++)
//		{
//			coloredLab.at<cv::Vec3b>(y, x)[0] = labColors[3 * labImg.at<cv::Vec3b>(y, x)[0]];
//			coloredLab.at<cv::Vec3b>(y, x)[1] = labColors[3 * labImg.at<cv::Vec3b>(y, x)[0] + 1];
//			coloredLab.at<cv::Vec3b>(y, x)[2] = labColors[3 * labImg.at<cv::Vec3b>(y, x)[0] + 2];
//			//labeling surroundings that have depth value
//			if ((labImg.at<cv::Vec3b>(y, x)[0] == 0) && disparityImg.at<unsigned short>(y, x) > 0)
//			{
//				labImg.at<cv::Vec3b>(y, x)[0] = 255;
//				labImg.at<cv::Vec3b>(y, x)[1] = 255;
//				labImg.at<cv::Vec3b>(y, x)[2] = 255;
//				//coloredLab.at<cv::Vec3b>(y, x)[0] = 255;
//				//coloredLab.at<cv::Vec3b>(y, x)[1] = 255;
//				//coloredLab.at<cv::Vec3b>(y, x)[2] = 255;
//			}
//		}
//	}
//	cv::imshow("Colored benchmark label", GenColoredGTImg(labImg));
//	cv::waitKey(1);
//
//	//Segmentation colored label image
//	cv::Mat coloredSegLab(480, 640, CV_8UC3, cv::Scalar::all(0));
//	Surfel *pCurrSurfel = surfels.NodeArray.Element;
//	RVL::QLIST::Index2 *pt;
//	unsigned char labSegColor[3];
//	int x = 0, y = 0;
//	int surfIdxOffset = 0;
//	for (int i = 0; i < surfels.NodeArray.n; pCurrSurfel++, i++)
//	{
//		pCurrSurfel->ObjectID = -1;
//		if ((pCurrSurfel->size == 1) || (pCurrSurfel->size == 0))
//			continue;
//		//Generate surfel color
//		labSegColor[0] = rand() % 255;
//		labSegColor[1] = rand() % 255;
//		labSegColor[2] = rand() % 255;
//		pt = pCurrSurfel->PtList.pFirst;
//		//Set pixel colors
//		for (int i = 0; i < pCurrSurfel->size; i++)
//		{
//			y = floor(pt->Idx / 640.0);
//			x = floor(pt->Idx - 640.0 * y);
//			coloredSegLab.at<cv::Vec3b>(y, x)[0] = labSegColor[0];
//			coloredSegLab.at<cv::Vec3b>(y, x)[1] = labSegColor[1];
//			coloredSegLab.at<cv::Vec3b>(y, x)[2] = labSegColor[2];
//			pt = pt->pNext;
//		}
//		pCurrSurfel->ObjectID = DetPrimaryGTObj(pCurrSurfel, labImg, 256/*maxLab + 1*/);
//		std::cout << pCurrSurfel->ObjectID << std::endl;
//	}
//	coloredSegLab = GenColoredSegmentationImg(&surfels);
//	//Connection
//	int x0 = 0, y0 = 0;
//	pCurrSurfel = surfels.NodeArray.Element;
//	Surfel *pOtherSurfel;
//	for (int i = 0; i < surfels.NodeArray.n; pCurrSurfel++, i++)
//	{
//		if ((pCurrSurfel->ObjectID == -1) || (pCurrSurfel->ObjectID == 255) || (pCurrSurfel->ObjectID == 0) || (pCurrSurfel->size == 1) || (pCurrSurfel->size == 0))
//			continue;
//		//Check surfel validity
//		if (GetSurfelGTValidity(pCurrSurfel, labImg, 5))
//		{
//			std::vector<Surfel*> nList;
//			//Get Surfel naighbours
//			FindSurfelNeighbours(nList, pCurrSurfel, &surfels, 6);
//			GetSurfelImgCentroid(x, y, pCurrSurfel);
//			for (int i = 0; i < nList.size(); i++)
//			{
//				pOtherSurfel = nList.at(i);
//				GetSurfelImgCentroid(x0, y0, pOtherSurfel);
//				cv::line(coloredSegLab, cv::Point(x, y), cv::Point(x0, y0), cv::Scalar::all(255));
//				AddVTKLine(pCurrSurfel->P, pOtherSurfel->P, visualizer.renderer);
//			}
//		}
//	}
//	cv::imshow("Colored segmentation label", coloredSegLab);
//	cv::waitKey(1);
//
//	visualizer.Run();
//}

//int main(int argc, char ** argv)
//{
//	//// Create memory storage.
//
//	//CRVLMem mem0;	// permanent memory
//
//	//mem0.Create(1000000);
//
//	//CRVLMem mem;	// cycle memory
//
//	//mem.Create(100000000);
//
//	//// Read parameters from a configuration file.
//
//	//char *MeshFileName = NULL;
//
//	//DWORD flags = 0x00000000;
//
//	//CRVLParameterList ParamList;
//
//	//CreateParamList(&ParamList, &mem0, &MeshFileName, flags);
//
//	//ParamList.LoadParams("RVLPCSegmentDemo.cfg");
//
//	//// Read mesh from file.
//
//	//PCLMeshBuilder meshBuilder;
//
//	//meshBuilder.CreateParamList(&mem0);
//
//	//meshBuilder.ParamList.LoadParams("RVLPCSegmentDemo.cfg");
//
//	//int w = 640;
//	//int h = 480;
//
//	//Mesh mesh;
//	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC(new pcl::PointCloud<pcl::PointXYZRGBA>(w, h));
//	//pcl::PolygonMesh PCLMesh;
//
//	//printf("Creating mesh from %s:\n", MeshFileName);
//
//	//if (mesh.Load(MeshFileName, &meshBuilder, PC, PCLMesh, (flags & RVLPCSEGMENT_DEMO_FLAG_SAVE_PLY) != 0))
//	//	printf("Mesh created.\n");
//	//else
//	//	printf("ERROR: Mesh can't be created!\n");
//	//
//	//// Segment mesh to surfels.
//
//	//SurfelGraph surfels;
//
//	//surfels.Init(&mesh);
//
//	//PlanarSurfelDetector detector;
//
//	//detector.CreateParamList(&mem0);
//
//	//detector.ParamList.LoadParams("RVLPCSegmentDemo.cfg");
//
//	//detector.Init(&mesh, &surfels, &mem);
//
//	//detector.pTimer = new CRVLTimer;
//
//	//printf("Segmentation to surfels...");
//
//	//double StartTime = detector.pTimer->GetTime();
//
//	//detector.Segment(&mesh, &surfels);
//
//	//double ExecTime = detector.pTimer->GetTime() - StartTime;
//
//	//printf("completed.\n");
//	//printf("No. of surfels = %d\n", surfels.NodeArray.n);
//	//printf("Total segmentation time = %lf s\n", ExecTime);
//
//	//// Display mesh.
//
//	//unsigned char SelectionColor[3];
//
//	//SelectionColor[0] = 0;
//	//SelectionColor[1] = 255;
//	//SelectionColor[2] = 0;
//
//	//surfels.NodeColors(SelectionColor);	
//
//	//Visualizer visualizer;	
//
//	//visualizer.Create();
//	//surfels.InitDisplay(&visualizer, &mesh, &detector);
//	//surfels.Display(&visualizer, &mesh);
//	//detector.DisplaySoftEdges(&visualizer, &mesh, &surfels, SelectionColor);
//	//visualizer.Run();
//
//	//// free memory
//
//	//delete detector.pTimer;
//	//delete[] MeshFileName;
//
//	RunSeg2Bench(true);
//	//SceneSegFile::SceneSegFile* ssf = new SceneSegFile::SceneSegFile("test");
//	///*ssf = SceneSegFile::GenerateTestSceneSegFile();
//	//ssf->Save("test.ssf");*/
//	//ssf->Load("test.ssf");
//
//	return 0;
//}