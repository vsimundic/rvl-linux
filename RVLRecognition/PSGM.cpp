//#include "stdafx.h"

#include "RVLVTK.h"
#include <vtkTriangle.h>
#include <vtkAxesActor.h>
#include <vtkLine.h>
#include "RVLCore2.h"
#include "Graph.h"
#include <Eigen\Eigenvalues>
#include <pcl/common/common.h>
#include <pcl/PolygonMesh.h>
#include "PCLTools.h"
#include "PCLMeshBuilder.h"
#include "RGBDCamera.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SurfelGraph.h"
#include "PlanarSurfelDetector.h"
#include "PSGM.h"

using namespace RVL;

PSGM::PSGM()
{
	surfelVertexList.Element = NULL;
	surfelVertexMem = NULL;
}


PSGM::~PSGM()
{
	RVL_DELETE_ARRAY(surfelVertexList.Element);
	RVL_DELETE_ARRAY(surfelVertexMem);
}

void PSGM::Interpret(
	Mesh *pMesh)
{
	// Create ordered mesh.

	pMesh->CreateOrderedMeshFromPolyData();

	// Detect surfels.

	pSurfels->Init(pMesh);

	pSurfelDetector->Init(pMesh, pSurfels, pMem);

	printf("Segmentation to surfels...");

	pSurfelDetector->Segment(pMesh, pSurfels);

	printf("completed.\n");

	int nSurfels = pSurfels->NodeArray.n;

	printf("No. of surfels = %d\n", nSurfels);

	// Detect vertices.

	QList<RECOG::PSGM_::Vertex> *pVertexList = &vertexList;

	RVLQLIST_INIT(pVertexList);

	int nVertexSurfelRelations = 0;

	RVL_DELETE_ARRAY(surfelVertexList.Element);

	surfelVertexList.Element = new QList<QLIST::Index>[pSurfels->NodeArray.n];
	surfelVertexList.n = pSurfels->NodeArray.n;

	int iSurfel, iSurfel_, iPrevSurfel;
	int iBoundary;
	int iPointEdge;
	int iPt, iPt_;
	Surfel *pSurfel;
	Array<MeshEdgePtr *> *pBoundary;
	MeshEdgePtr *pEdgePtr, *pEdgePtr_;
	MeshEdge *pEdge;
	QList<MeshEdgePtr> *pEdgeList;
	RECOG::PSGM_::Vertex *pVertex;
	Point *pPt;
	QList<QLIST::Index> *pSurfelVertexList;

	for (iSurfel = 0; iSurfel < pSurfels->NodeArray.n; iSurfel++)
	{
		pSurfelVertexList = surfelVertexList.Element + iSurfel;

		RVLQLIST_INIT(pSurfelVertexList);

		pSurfel = pSurfels->NodeArray.Element + iSurfel;

		if (pSurfel->size == 0)
			continue;

		for (iBoundary = 0; iBoundary < pSurfel->BoundaryArray.n; iBoundary++)
		{
			pBoundary = pSurfel->BoundaryArray.Element + iBoundary;

			for (iPointEdge = 0; iPointEdge < pBoundary->n; iPointEdge++)
			{
				pEdgePtr = pBoundary->Element[iPointEdge];

				pEdgePtr_ = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pEdgePtr);

				iPt = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr_);

				pPt = pMesh->NodeArray.Element + iPt;

				pEdgeList = &(pPt->EdgeList);

				iPrevSurfel = -1;

				//int debug = 0;

				while (true)
				{					
					RVLQLIST_GET_NEXT_CIRCULAR(pEdgeList, pEdgePtr_);

					RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iPt, pEdgePtr_, pEdge, iPt_);

					iSurfel_ = pSurfels->surfelMap[iPt_];

					if (iSurfel_ == iSurfel)
						break;

					//debug++;

					//if (debug >= 20)
					//	debug = 0;

					if (iSurfel_ >= 0 && iSurfel_ < pMesh->NodeArray.n)
					{
						if (iPrevSurfel >= 0 && iPrevSurfel != iSurfel_)
						{
							if (iSurfel < iSurfel_ && iSurfel < iPrevSurfel)
							{
								RVLMEM_ALLOC_STRUCT(pMem, RECOG::PSGM_::Vertex, pVertex);

								RVLCOPY3VECTOR(pPt->P, pVertex->P);

								RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, 3, pVertex->iSurfelArray.Element);

								pVertex->iSurfelArray.Element[0] = iSurfel;
								pVertex->iSurfelArray.Element[1] = iSurfel_;
								pVertex->iSurfelArray.Element[2] = iPrevSurfel;
								pVertex->iSurfelArray.n = 3;

								RVLQLIST_ADD_ENTRY(pVertexList, pVertex);

								nVertexSurfelRelations += 3;
							}
						}

						iPrevSurfel = iSurfel_;
					}	// if (iSurfel_ >= 0 && iSurfel_ < pMesh->NodeArray.n)					
				}	// for each neighboring point of the point iPt
			}	// for each point-edge on the boundary contour
		}	// for each boundary contour
	}	// for each surfel

	RVL_DELETE_ARRAY(surfelVertexMem);

	surfelVertexMem = new QLIST::Index[nVertexSurfelRelations];

	QLIST::Index *pVertexIdx = surfelVertexMem;

	int iVertex = 0;

	pVertex = vertexList.pFirst;

	while (pVertex)
	{
		for (iSurfel = 0; iSurfel < pVertex->iSurfelArray.n; iSurfel++)
		{
			pSurfelVertexList = surfelVertexList.Element + iSurfel;

			RVLQLIST_ADD_ENTRY(pSurfelVertexList, pVertexIdx);

			pVertexIdx->Idx = iVertex;

			pVertexIdx++;
		}

		iVertex++;

		pVertex = pVertex->pNext;
	}

	nVertices = iVertex;
}

void PSGM::InitDisplay(
	Visualizer *pVisualizer,
	Mesh *pMesh)
{
	pVisualizer->SetMesh(pMesh);

	displayData.pMesh = pMesh;
	displayData.pSurfels = pSurfels;
	displayData.pRecognition = this;
	displayData.pVisualizer = pVisualizer;

	pSurfels->InitDisplay(pVisualizer, pMesh, pSurfelDetector);
}

void PSGM::Display()
{
	Mesh *pMesh = displayData.pMesh;
	Visualizer *pVisualizer = displayData.pVisualizer;

	pSurfels->Display(pVisualizer, pMesh);

	DisplayVertices();
}

void PSGM::DisplayModelInstance(Visualizer *pVisualizer)
{
	//Create polygonal mesh

	//// Setup four points
	vtkSmartPointer<vtkPoints> points =
		vtkSmartPointer<vtkPoints>::New();
	points->InsertNextPoint(0.0, 0.0, 0.0);
	points->InsertNextPoint(1.0, 0.0, 0.0);
	points->InsertNextPoint(1.5, 0.5, 0.0);
	points->InsertNextPoint(1.0, 1.0, 0.0);
	points->InsertNextPoint(0.0, 1.0, 0.0);

	// Define some colors
	unsigned char red[3] = { 255, 0, 0 };
	unsigned char green[3] = { 0, 255, 0 };
	unsigned char blue[3] = { 0, 0, 255 };
	unsigned char white[3] = { 255, 255, 255 };
	unsigned char black[3] = { 0, 0, 0 };

	// Setup the colors array
	vtkSmartPointer<vtkUnsignedCharArray> colors =
		vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->SetName("Colors");

	// Add the three colors we have created to the array
	colors->InsertNextTupleValue(red);
	colors->InsertNextTupleValue(green);
	colors->InsertNextTupleValue(blue);
	colors->InsertNextTupleValue(white);
	colors->InsertNextTupleValue(black);

	// Create the polygon
	vtkSmartPointer<vtkPolygon> polygon =
		vtkSmartPointer<vtkPolygon>::New();
	polygon->GetPointIds()->SetNumberOfIds(5); //make a quad
	polygon->GetPointIds()->SetId(0, 0);
	polygon->GetPointIds()->SetId(1, 1);
	polygon->GetPointIds()->SetId(2, 2);
	polygon->GetPointIds()->SetId(3, 3);
	polygon->GetPointIds()->SetId(4, 4);
	//polygon->GetPointIds()->SetId(5, 5);

	// Add the polygon to a list of polygons
	vtkSmartPointer<vtkCellArray> polygons =
		vtkSmartPointer<vtkCellArray>::New();
	polygons->InsertNextCell(polygon);

	// Create a polydata object and add everything to it
	vtkSmartPointer<vtkPolyData> polydata =
		vtkSmartPointer<vtkPolyData>::New();
	polydata->SetPoints(points);
	polydata->SetPolys(polygons);
	polydata->GetPointData()->SetScalars(colors);

	//Mapper
	pVisualizer->map = vtkSmartPointer<vtkPolyDataMapper>::New();
	//map->SetInputData(pMesh->pPolygonData);
	pVisualizer->map->SetInputData(polydata);
	//map->SetInputConnection(polyDataNormals->GetOutputPort());
	pVisualizer->map->InterpolateScalarsBeforeMappingOff();

	//Actor
	pVisualizer->actor = vtkSmartPointer<vtkActor>::New();
	pVisualizer->actor->SetMapper(pVisualizer->map);

	//Insert actor
	pVisualizer->renderer->AddActor(pVisualizer->actor);
}

void PSGM::DisplayVertices()
{
	double lineLength = 10.0;

	Mesh *pMesh = displayData.pMesh;
	Visualizer *pVisualizer = displayData.pVisualizer;

	// Create the polydata where we will store all the geometric data
	vtkSmartPointer<vtkPolyData> linesPolyData =
		vtkSmartPointer<vtkPolyData>::New();

	// Create a vtkPoints container and store the points in it
	vtkSmartPointer<vtkPoints> pts =
		vtkSmartPointer<vtkPoints>::New();

	int iLine = 0;

	double P0[3], P[3], V[3];
	Surfel *pSurfel;
	int iSurfel;

	RECOG::PSGM_::Vertex *pVertex = vertexList.pFirst;

	while (pVertex)
	{
		RVLCOPY3VECTOR(pVertex->P, P0);

		for (iSurfel = 0; iSurfel < pVertex->iSurfelArray.n; iSurfel++)
		{
			pSurfel = pSurfels->NodeArray.Element + pVertex->iSurfelArray.Element[iSurfel];

			pts->InsertNextPoint(P0);

			RVLSCALE3VECTOR(pSurfel->N, lineLength, V);

			RVLSUM3VECTORS(P0, V, P);

			pts->InsertNextPoint(P);

			iLine++;
		}

		pVertex = pVertex->pNext;
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

	iLine = 0;

	pVertex = vertexList.pFirst;

	while (pVertex)
	{
		for (iSurfel = 0; iSurfel < pVertex->iSurfelArray.n; iSurfel++)
		{
			line[iLine] = vtkSmartPointer<vtkLine>::New();

			line[iLine]->GetPointIds()->SetId(0, 2 * iLine);
			line[iLine]->GetPointIds()->SetId(1, 2 * iLine + 1);

			lines->InsertNextCell(line[iLine]);

			colors->InsertNextTupleValue(red);

			iLine++;
		}

		pVertex = pVertex->pNext;
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

	vtkSmartPointer<vtkActor> actor =
		vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);

	pVisualizer->renderer->AddActor(actor);

	delete[] line;
}