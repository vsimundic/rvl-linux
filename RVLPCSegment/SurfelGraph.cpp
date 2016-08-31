//#include "stdafx.h"
#include "RVLVTK.h"
#include <vtkPolyLine.h>
#include "RVLCore2.h"
#include "Util.h"
#include "Graph.h"
//#include <Eigen\Eigenvalues>
//#include <pcl/common/common.h>
//#include <pcl/PolygonMesh.h>
//#include "PCLTools.h"
//#include "PCLMeshBuilder.h"
//#include "RGBDCamera.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SurfelGraph.h"
#include "PlanarSurfelDetector.h"
//#include "RFRecognition.h" //VIDOVIC
//#include <Eigen\Eigenvalues>

using namespace RVL;

SurfelGraph::SurfelGraph()
{
	PtMem = NULL;
	surfelBndMem = NULL;
	surfelBndMem2 = NULL;
	neighborEdge = NULL;
	surfelMap = NULL;
	edgeMap = NULL;
	nodeColor = NULL;
	NodeArray.Element = NULL;
	edgeMarkMap = NULL;
	EdgeArray.Element = NULL;
	DisplayData.mouseRButtonDownUserFunction = NULL;
	DisplayData.keyPressUserFunction = NULL;
}


SurfelGraph::~SurfelGraph()
{
	Clear();
}

void SurfelGraph::CreateParamList(CRVLMem *pMem)
{
	ParamList.m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	ParamList.Init();

	pParamData = ParamList.AddParam("SurfelGraph.visualization.edgeFeatureDepth", RVLPARAM_TYPE_FLOAT, &(DisplayData.edgeFeatureDepth));
}

void SurfelGraph::InitGetNeighborsBoundaryAndSize()
{
	neighborEdge = new SURFEL::Edge *[NodeArray.n];

	memset(neighborEdge, 0, NodeArray.n * sizeof(SURFEL::Edge *));

	memset(edgeMarkMap, 0, nMeshEdges * sizeof(unsigned char));
}

void SurfelGraph::FreeGetNeighborsBoundaryAndSize()
{
	RVL_DELETE_ARRAY(neighborEdge);
}

void SURFEL::ComputeParameters(
	Surfel *pSurfel,
	MESH::Distribution &distribution,
	Point *pPt)
{
	float *var = distribution.var;

	int idx[3];
	int iTmp;

	RVLSORT3DESCEND(var, idx, iTmp);

	float *N = pSurfel->N;
	float *N_ = distribution.R + 3 * idx[0];

	RVLCOPY3VECTOR(N_, N);

	if (RVLDOTPRODUCT3(pPt->N, N) > 0.0)
	{
		RVLCOPY3VECTOR(N, pSurfel->N)
	}
	else
	{
		RVLNEGVECT3(N, pSurfel->N)
	}

	float *P = pSurfel->P;
	float *P_ = distribution.t;

	RVLCOPY3VECTOR(P_, P);

	pSurfel->d = RVLDOTPRODUCT3(N, P);

	int *RGB = pSurfel->RGB;
	int *RGB_ = distribution.RGB;

	RVLCOPY3VECTOR(RGB_, RGB);

	float *P0 = pSurfel->P0;

	RVLCOPY3VECTOR(pPt->P, P0);

	pSurfel->r0 = pSurfel->d / RVLDOTPRODUCT3(N, P0);
}

void SURFEL::CreateFromPoint(
	Surfel *pSurfel, 
	Point *pPt)
{
	float *P = pSurfel->P;
	float *P_ = pPt->P;

	RVLCOPY3VECTOR(P_, P);

	float *P0 = pSurfel->P0;

	RVLCOPY3VECTOR(P_, P0);

	float *N = pSurfel->N;
	float *N_ = pPt->N;

	RVLCOPY3VECTOR(N_, N);

	pSurfel->d = RVLDOTPRODUCT3(N, P);

	RVLCOPY3VECTOR(pPt->RGB, pSurfel->RGB);

	pSurfel->r0 = pSurfel->d / RVLDOTPRODUCT3(N, P0);
}

// Create point pPoint from the surfel pSurfel such that its position is identical to the position of the surfel centroid,
// its normal i identical to the surfel normal and its color is identical to the surfel color

void SURFEL::GetPoint(
	Surfel *pSurfel,
	Point *pPoint)
{
	RVLCOPY3VECTOR(pSurfel->P, pPoint->P);
	RVLCOPY3VECTOR(pSurfel->N, pPoint->N);
	RVLCOPY3VECTOR(pSurfel->RGB, pPoint->RGB);
}

void SurfelGraph::Init(Mesh *pMesh)
{
	Clear();

	nMeshVertices = pMesh->NodeArray.n;
	nMeshEdges = pMesh->EdgeArray.n;

	PtMem = new QLIST::Index2[nMeshVertices];
	surfelBndMem = new MeshEdgePtr *[2 * nMeshEdges];
	surfelBndMem2 = new Array<MeshEdgePtr *>[nMeshEdges];
	surfelMap = new int[nMeshVertices];
	edgeMap = new int[nMeshVertices];
	//surfelBndMap = new QLIST::Index2[nPoints];
	NodeArray.Element = new Surfel[2 * nMeshVertices];
	edgeMarkMap = new unsigned char[nMeshEdges];
}


void SurfelGraph::Clear()
{
	RVL_DELETE_ARRAY(PtMem);
	RVL_DELETE_ARRAY(surfelBndMem);
	RVL_DELETE_ARRAY(surfelBndMem2);
	RVL_DELETE_ARRAY(surfelMap);
	RVL_DELETE_ARRAY(edgeMap);	
	//RVL_DELETE_ARRAY(surfelBndMap);
	RVL_DELETE_ARRAY(nodeColor);
	RVL_DELETE_ARRAY(NodeArray.Element);
	RVL_DELETE_ARRAY(edgeMarkMap);
	RVL_DELETE_ARRAY(neighborEdge);
	RVL_DELETE_ARRAY(EdgeArray.Element);
}

void SurfelGraph::NodeColors(unsigned char *SelectionColor)
{
	int SelectionColor_[3];

	RVLCONVTOINT3(SelectionColor, SelectionColor_);

	RVL_DELETE_ARRAY(nodeColor);

	nodeColor = new unsigned char[3 * NodeArray.n];

	Surfel *pSurfel;
	int iNode;
	int Color[3], dColor[3];
	unsigned char *NodeColor_;

	for (iNode = 0; iNode < NodeArray.n; iNode++)
	{
		pSurfel = NodeArray.Element + iNode;

		do
		{
			Color[0] = rand() % 256;
			Color[1] = rand() % 256;
			Color[2] = rand() % 256;

			RVLDIF3VECTORS(Color, SelectionColor, dColor);
		} while (RVLDOTPRODUCT3(dColor, dColor) < 128 * 128);

		NodeColor_ = nodeColor + 3 * iNode;

		NodeColor_[0] = (unsigned char)Color[0];
		NodeColor_[1] = (unsigned char)Color[1];
		NodeColor_[2] = (unsigned char)Color[2];
	}
}

void SurfelGraph::DisplayHardEdges(
	Visualizer *pVisualizer,
	Mesh *pMesh,
	int iSurfel,
	unsigned char *Color)
{
	Surfel *pSurfel = NodeArray.Element + iSurfel;

	QLIST::Index2 *pPtIdx = pSurfel->PtList.pFirst;

	int iPt, iPt_;
	MeshEdgePtr *pEdgePtr;
	MeshEdge *pEdge;
	int iSurfel_;
	//Surfel *pSurfel_;
	bool bEdge;
	Point *pPt;

	while (pPtIdx)
	{
		iPt = pPtIdx->Idx;

		pPt = pMesh->NodeArray.Element + iPt;

		bEdge = false;

		pEdgePtr = pPt->EdgeList.pFirst;

		while (pEdgePtr)
		{
			RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iPt, pEdgePtr, pEdge, iPt_);

			iSurfel_ = surfelMap[iPt_];

			if (iSurfel_ != iSurfel)
			{
				bEdge = true;

				break;
			}

			pEdgePtr = pEdgePtr->pNext;
		}

		if (bEdge)
			pVisualizer->PaintPoint(iPt, pMesh->pPolygonData, Color);
		//else
		//	int debug = 0;

		pPtIdx = pPtIdx->pNext;
	}
}

void SurfelGraph::Display(
	Visualizer *pVisualizer,
	Mesh *pMesh,
	int iSelectedSurfel,
	unsigned char *SelectionColor,
	int *ColorScale,
	unsigned char *ColorOffset)
{
	//unsigned char HardEdgeColor[3];

	//HardEdgeColor[0] = 0;
	//HardEdgeColor[1] = 255;
	//HardEdgeColor[2] = 0;

	int iSurfel;
	Surfel *pSurfel;
	unsigned char color[3];
	unsigned char *color_;

	for (iSurfel = 0; iSurfel < NodeArray.n; iSurfel++)
	{
		pSurfel = NodeArray.Element + iSurfel;

		if (pSurfel->bEdge)
			continue;

		color_ = nodeColor + 3 * iSurfel;

		if (iSurfel == iSelectedSurfel)
			pVisualizer->PaintPointSet(&(pSurfel->PtList), pMesh->pPolygonData, SelectionColor);
		else
		{
			RVLCOPY3VECTOR(color_, color);

			if (ColorScale)
				RVLSCALECOLOR2(color, ColorScale, color);

			if (ColorOffset)
			{
				RVLSUM3VECTORS(color, ColorOffset, color);
			}

			pVisualizer->PaintPointSet(&(pSurfel->PtList), pMesh->pPolygonData, color);
		}
		
		//DisplayHardEdges(pVisualizer, pMesh, iSurfel, HardEdgeColor);
	}

	DisplayEdgeFeatures();
}

//VTK Render window right mouse button press callback
void SURFEL::MouseRButtonDown(vtkObject* caller, unsigned long eid, void* clientdata, void *calldata)
{
	vtkSmartPointer<vtkRenderWindowInteractor> interactor = reinterpret_cast<vtkRenderWindowInteractor*>(caller);
	SURFEL::DisplayCallbackData *pData = (SURFEL::DisplayCallbackData *)clientdata;

	Mesh *pMesh = pData->pMesh;

	vtkSmartPointer<vtkPolyData> pd = pMesh->pPolygonData;

	vtkSmartPointer<vtkFloatArray> pointData;
	vtkSmartPointer<vtkUnsignedCharArray> rgbPointData;
	vtkSmartPointer<vtkFloatArray> normalPointData;
	int noPts = 0;
	//FetchVTKPointData(pd, pointData, rgbPointData, normalPointData, noPts);

	pData->pVisualizer->pointPicker->Pick(interactor->GetEventPosition()[0], interactor->GetEventPosition()[1], 0, 
		interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
	vtkIdType selectedPoint = pData->pVisualizer->pointPicker->GetPointId();

	if (selectedPoint >= 0)
	{
		int iSurfel = pData->pSurfels->surfelMap[selectedPoint];

		bool bSelection = false;

		if (pData->mouseRButtonDownUserFunction)
		{
			if (iSurfel >= 0)
				bSelection |= pData->mouseRButtonDownUserFunction(pMesh, pData->pSurfels, (int)selectedPoint, iSurfel, pData->vpUserFunctionData);
		}

		if (!bSelection)
		{
			if (pData->iSelectedSurfel >= 0 && (pData->iSelection == 1 && pData->iSelectedSurfel != iSurfel))
				pData->pVisualizer->PaintPointSet(&(pData->pSurfels->NodeArray.Element[pData->iSelectedSurfel].PtList), pMesh->pPolygonData,
				pData->pSurfels->GetColor(pData->iSelectedSurfel));

			if (pData->iSelectedSurfel2 >= 0 && ((pData->iSelection == 1 || (pData->iSelection == 2 && pData->iSelectedSurfel2 != iSurfel))))
				pData->pVisualizer->PaintPointSet(&(pData->pSurfels->NodeArray.Element[pData->iSelectedSurfel2].PtList), pMesh->pPolygonData,
				pData->pSurfels->GetColor(pData->iSelectedSurfel2));

			//pData->pSurfels->DisplaySurfelBoundary(pData->pVisualizer, pMesh, iSurfel, pData->SelectionColor);

			if (pData->iSelection == 1)
			{
				if (iSurfel >= 0)
					pData->pVisualizer->PaintPointSet(&(pData->pSurfels->NodeArray.Element[iSurfel].PtList), pMesh->pPolygonData, pData->SelectionColor);

				pData->iSelectedSurfel = iSurfel;

				pData->iSelectedSurfel2 = -1;
			}
			else// if (pData->iSelection == 2)
			{
				unsigned char SelectionColor2[3];

				RVLSCALECOLOR(pData->SelectionColor, 75, SelectionColor2);

				pData->pVisualizer->PaintPointSet(&(pData->pSurfels->NodeArray.Element[iSurfel].PtList), pMesh->pPolygonData, SelectionColor2);

				pData->iSelectedSurfel2 = iSurfel;

				pData->iSelection = 1;
			}

			bSelection = true;
		}

		if (bSelection)
		{
			pd->Modified();

			pData->pSurfels->PrintData(pData->pVisualizer, pMesh, selectedPoint, iSurfel);

			interactor->GetRenderWindow()->Render();
		}
	}
}

//VTK Render window key press callback
void SURFEL::KeyPressCallback(vtkObject* caller, unsigned long eid, void* clientdata, void *calldata)
{
	vtkSmartPointer<vtkRenderWindowInteractor> interactor = reinterpret_cast<vtkRenderWindowInteractor*>(caller);
	SURFEL::DisplayCallbackData *pData = (SURFEL::DisplayCallbackData *)clientdata;

	Mesh *pMesh = pData->pMesh;

	vtkSmartPointer<vtkPolyData> pd = pMesh->pPolygonData;

	vtkSmartPointer<vtkFloatArray> pointData;
	vtkSmartPointer<vtkUnsignedCharArray> rgbPointData;
	vtkSmartPointer<vtkFloatArray> normalPointData;
	int noPts = 0;

	std::string keySym = "";
	keySym = interactor->GetKeySym();

	PlanarSurfelDetector *pDetector = (PlanarSurfelDetector *)(pData->vpDetector);

	bool bUpdateDisplay = false;
	bool bDisplayBoundary = false;
	bool bDefineBoundary = false;

	if (keySym == "2")
	{
		pData->iSelection = 2;
	}
	else if(keySym == "b")
	{
		if (pData->mode != RVLSURFEL_DISPLAY_MODE_NEIGHBOR_PAIR)
		{
			int colorScale[3];

			RVLSET3VECTOR(colorScale, 0, 0, 75);

			unsigned char colorOffset[3];

			RVLSET3VECTOR(colorOffset, 0, 0, 64);

			pData->pSurfels->Display(pData->pVisualizer, pMesh, -1, NULL, colorScale, colorOffset);

			bDefineBoundary = true;

			bUpdateDisplay = true;
		}
	}
	else if (keySym == "c")
	{
		bDisplayBoundary = true;

		bUpdateDisplay = true;
	}
	else if (keySym == "n")
	{
		if (pData->pVisualizer->bNormals)
		{
			pData->pVisualizer->bNormalsVisible = !pData->pVisualizer->bNormalsVisible;

			if (pData->pVisualizer->bNormalsVisible)
				pData->pVisualizer->normals->VisibilityOn();
			else
				pData->pVisualizer->normals->VisibilityOff();

			bUpdateDisplay = true;
		}
	}
	else if (keySym == "p")
	{
		if (pData->iSelectedSurfel >= 0)
		{
			pDetector->DefinePolygon(pMesh, pData->pSurfels, pData->iSelectedSurfel);

			pData->pSurfels->Display(pData->pVisualizer, pMesh, pData->iSelectedSurfel, pData->SelectionColor);

			bUpdateDisplay = true;
		}
	}
	else if (keySym == "s")
	{
		if (pData->mode == RVLSURFEL_DISPLAY_MODE_NEIGHBOR_PAIR)
		{
			pData->pSurfels->Display(pData->pVisualizer, pMesh, pData->iSelectedSurfel, pData->SelectionColor);

			bUpdateDisplay = true;
		}

		pData->mode = RVLSURFEL_DISPLAY_MODE_SURFELS;
	}
#ifdef RVLMESH_BOUNDARY_DEBUG
	else if (keySym == "plus")
	{
		if (pData->mode == RVLSURFEL_DISPLAY_MODE_NEIGHBOR_PAIR)
		{
			if (pData->iSelectedSurfel >= 0 && pData->iSelectedSurfel2 >= 0)
			{
				pMesh->debugState++;

				bDefineBoundary = true;

				bUpdateDisplay = true;
			}
		}
		else
		{
			if (pData->iSelectedSurfel >= 0)
			{
				pMesh->debugState++;

				bDisplayBoundary = true;

				bUpdateDisplay = true;
			}
		}
	}
#endif

	if (pData->keyPressUserFunction)
		bUpdateDisplay |= pData->keyPressUserFunction(pMesh, pData->pSurfels, keySym, pData->vpUserFunctionData);

	if (bDefineBoundary)
	{	
		QList<QLIST::Index> G;

		int iSurfel = pData->iSelectedSurfel;
		int iSurfel_ = pData->iSelectedSurfel2;

		pDetector->DefineBoundaryTest(pMesh, pData->pSurfels, iSurfel, iSurfel_, G);

		unsigned char white[3];

		RVLSET3VECTOR(white, 255, 255, 255);

		unsigned char green[3];

		RVLSET3VECTOR(green, 0, 255, 0);

		unsigned char black[3];

		RVLSET3VECTOR(black, 0, 0, 0);

		pData->pVisualizer->PaintPointSet(&(pData->pSurfels->NodeArray.Element[iSurfel].PtList), pMesh->pPolygonData, white);

		//pData->pVisualizer->PaintPointSet(&G, pMesh->pPolygonData, green);

		pData->pVisualizer->PaintPointSet(&(pData->pSurfels->NodeArray.Element[iSurfel_].PtList), pMesh->pPolygonData, black);

#ifdef RVLPLANARSURFELDETECTOR_CONNECTED_COMPONENT_DEBUG
		unsigned char red[3];

		RVLSET3VECTOR(red, 255, 0, 0);

		pData->pVisualizer->PaintPointSet(&(pDetector->debugPtArray), pMesh->pPolygonData, red);
#endif

//#ifdef RVLPLANARSURFELDETECTOR_EDGE_BOUNDARY_DEBUG
//		pData->pVisualizer->PaintPointSet(&(pDetector->debugPtArray), pMesh->pPolygonData, green);
//#endif

		pData->mode = RVLSURFEL_DISPLAY_MODE_NEIGHBOR_PAIR;
	}
	
	if (bDisplayBoundary)
	{
		if (pData->iSelectedSurfel >= 0)
		{
			pData->pVisualizer->PaintPointSet(&(pData->pSurfels->NodeArray.Element[pData->iSelectedSurfel].PtList), pMesh->pPolygonData,
				pData->pSurfels->GetColor(pData->iSelectedSurfel));

			FILE *fpPts = fopen("C:\\RVL\\Debug\\PSDEdgeBoundaryDebugPoints.txt", "w");
			FILE *fpEdges = fopen("C:\\RVL\\Debug\\PSDEdgeBoundaryDebugEdges.txt", "w");

			pData->pSurfels->Save(pData->iSelectedSurfel, pMesh, fpPts, fpEdges);

			fclose(fpPts);
			fclose(fpEdges);

			pData->pSurfels->DisplaySurfelBoundary(pData->pVisualizer, pMesh, pData->iSelectedSurfel, pData->SelectionColor);
		}
	}

	if (bUpdateDisplay)
	{
		pd->Modified();

		pData->pSurfels->PrintData(pData->pVisualizer, pMesh, -1, pData->iSelectedSurfel);

		interactor->GetRenderWindow()->Render();
	}
}

void SurfelGraph::PrintData(
	Visualizer *pVisualizer,
	Mesh *pMesh,
	int iVertex,
	int iSurfel)
{
	Surfel *pSurfel = NodeArray.Element + iSurfel;

	char str[2000], str2[500];

	if (iVertex >= 0)
	{
		Point *pPt = pMesh->NodeArray.Element + iVertex;

		sprintf(str, "Point %d\nP=(%f, %f, %f)\nN=(%f, %f, %f)\nRGB=(%d, %d, %d)",
			iVertex, pPt->P[0], pPt->P[1], pPt->P[2], pPt->N[0], pPt->N[1], pPt->N[2], pPt->RGB[0], pPt->RGB[1], pPt->RGB[2]);
	}
	else
		str[0] = 0;

	if (iSurfel >= 0)
	{
		sprintf(str2, "\nSurfel %d\nP=(%f, %f, %f)\nN=(%f, %f, %f)\nRGB=(%d, %d, %d)\nsize=%d",
			iSurfel,
			pSurfel->P[0], pSurfel->P[1], pSurfel->P[2],
			pSurfel->N[0], pSurfel->N[1], pSurfel->N[2],
			pSurfel->RGB[0], pSurfel->RGB[1], pSurfel->RGB[2],
			pSurfel->size);

		strcat(str, str2);
	}

	//// Print indices of the adjacent surfels

	//strcat(str, "\nNeighbors:\n");

	//VertexEdgePtr *pEdgePtr = pSurfel->EdgeList.pFirst;

	//Surfel *pSurfel_ = pSurfel;

	//Surfel *pSurfel__;
	//MeshEdge *pEdge;
	//int iSurfel__;
	//float eZ_, eZ__, eXY;
	//float N_[3], N__[3], Z[3];
	//float V3Tmp[3];
	//int RGB_[3], RGB__[3], dRGB[3], eRGB;
	//float fTmp;

	//RVLCONVTOINT3(pSurfel_->RGB, RGB_);

	//while (pEdgePtr)	// for each neighbor of iNode
	//{
	//	RVLSEGMENTATION_GET_NEIGHBOR(iNode, pEdgePtr, pEdge, iSurfel__);

	//	pSurfel__ = surfelArray.Element + iSurfel__;

	//	RVLCONVTOINT3(pSurfel__->RGB, RGB__);

	//	RVLDIF3VECTORS(RGB__, RGB_, dRGB);

	//	eRGB = RVLDOTPRODUCT3(dRGB, dRGB);

	//	RVLDIF3VECTORS(pSurfel__->P, pSurfel_->P, Z);

	//	RVLNORM3(Z, fTmp);

	//	eZ_ = RVLDOTPRODUCT3(Z, pSurfel_->N);

	//	eZ__ = RVLDOTPRODUCT3(Z, pSurfel__->N);

	//	RVLSCALE3VECTOR(Z, eZ_, V3Tmp);
	//	RVLDIF3VECTORS(pSurfel_->N, V3Tmp, N_);
	//	RVLNORM3(N_, fTmp);
	//	RVLSCALE3VECTOR(Z, eZ__, V3Tmp);
	//	RVLDIF3VECTORS(pSurfel__->N, V3Tmp, N__);
	//	RVLNORM3(N__, fTmp);
	//	RVLDIF3VECTORS(N__, N_, V3Tmp);

	//	eXY = RVLDOTPRODUCT3(V3Tmp, V3Tmp);

	//	sprintf(str2, "%d: e=(%f, %f, %f, %f)\n", iSurfel__, eZ_, eZ__, sqrt(eXY), sqrt((float)eRGB));

	//	strcat(str, str2);

	//	pEdgePtr = pEdgePtr->pNext;
	//}	// for each neighbor of iNode

	// Put the text on the screen

	pVisualizer->text->SetText(2, str);
}


unsigned char * SurfelGraph::GetColor(int iSurfel)
{
	return nodeColor + 3 * iSurfel;
}


void SurfelGraph::InitDisplay(
	Visualizer *pVisualizer,
	Mesh *pMesh,
	void *vpDetector)
{
	pVisualizer->SetMesh(pMesh);

	DisplayData.pMesh = pMesh;
	DisplayData.pSurfels = this;
	DisplayData.pVisualizer = pVisualizer;
	DisplayData.vpDetector = vpDetector;
	RVLSET3VECTOR(DisplayData.SelectionColor, 0, 255, 0);
	DisplayData.mode = RVLSURFEL_DISPLAY_MODE_SURFELS;
	DisplayData.iSelectedSurfel = DisplayData.iSelectedSurfel2 = -1;
	DisplayData.iSelection = 1;

	pVisualizer->SetMouseRButtonDownCallback(SURFEL::MouseRButtonDown, &DisplayData);
	pVisualizer->SetKeyPressCallback(SURFEL::KeyPressCallback, &DisplayData);
}


void SurfelGraph::DisplaySurfelBoundary(
	Visualizer *pVisualizer, 
	Mesh * pMesh, 
	int iSurfel,
	unsigned char *Color)
{
	Surfel *pSurfel = NodeArray.Element + iSurfel;

	//QList<QLIST::Index2> *pSurfelPtList = &(pSurfel->PtList);

	//pSurfel->BoundaryArray.Element = new Array <MeshEdgePtr *>[nMeshVertices];

	//MeshEdgePtr **boundaryMem = new MeshEdgePtr *[pMesh->EdgeArray.n];

	//MeshEdgePtr **pBoundaryMem = boundaryMem;

	//pMesh->Boundary(pSurfelPtList, surfelMap, pSurfel->BoundaryArray, pBoundaryMem, edgeMarkMap);

	Array<int> boundaryPtArray;

	boundaryPtArray.Element = new int[nMeshVertices];
	boundaryPtArray.n = 0;

	int iBoundary, iPointEdge; 
	Array<MeshEdgePtr *> *pBoundary;
	MeshEdgePtr *pEdgePtr;

	for (iBoundary = 0; iBoundary < pSurfel->BoundaryArray.n; iBoundary++)
	{
		pBoundary = pSurfel->BoundaryArray.Element + iBoundary;

		for (iPointEdge = 0; iPointEdge < pBoundary->n; iPointEdge++)
		{
			pEdgePtr = pBoundary->Element[iPointEdge];

			boundaryPtArray.Element[boundaryPtArray.n++] = RVLPCSEGMENT_GRAPH_GET_NODE(pEdgePtr);
		}
	}

	pVisualizer->PaintPointSet(&boundaryPtArray, pMesh->pPolygonData, Color);

	//delete[] pSurfel->BoundaryArray.Element;
	//delete[] boundaryMem;
	delete[] boundaryPtArray.Element;

	//QList<QLIST::Index> Boundary;

	//QLIST::Index *BoundaryMem = new QLIST::Index[pMesh->NodeArray.n];

	//pMesh->Boundary(pSurfelPtList, surfelMap, &Boundary, BoundaryMem);

	//pVisualizer->PaintPointSet(&Boundary, pMesh->pPolygonData, Color);

	//delete[] BoundaryMem;
}

void SurfelGraph::DisplayEdgeFeatures()
{
	Visualizer *pVisualizer = DisplayData.pVisualizer;

	// Create the polydata where we will store all the geometric data
	DisplayData.edgeFeaturesPolyData = vtkSmartPointer<vtkPolyData>::New();

	// Create a vtkPoints container and store the points in it
	vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();

	//// Create a cell array to store the lines in and add the lines to it
	vtkSmartPointer<vtkCellArray> polyLines = vtkSmartPointer<vtkCellArray>::New();

#ifdef NEVER
	// Create colors.
	vtkSmartPointer<vtkUnsignedCharArray> colors =
		vtkSmartPointer<vtkUnsignedCharArray>::New();	

	colors->SetNumberOfComponents(3);

	unsigned char red[3] = { 255, 0, 0 };

	colors->InsertNextTupleValue(red);
#endif
	///

	// Determine the total number of edge features.

	int nEdgeFeatures = 0;

	int iFeature;

	for (iFeature = 0; iFeature < NodeArray.n; iFeature++)
		if (NodeArray.Element[iFeature].bEdge)
			nEdgeFeatures++;

	// Allocate polyline pointers.

	vtkSmartPointer<vtkPolyLine> *polyLine = new vtkSmartPointer<vtkPolyLine>[nEdgeFeatures];

	//

	int iEdgeFeature = 0;

	int i, iParentSurfel;
	Surfel *pFeature, *pParentSurfel;
	SURFEL::EdgePtr *pSEdgePtr;
	float *NParent, *N, *P1;
	float P2[3], P3[3], P4[3], U[3], V[3], VTmp[3];
	//Eigen::Matrix3f M;
	//Eigen::Vector3f B, t_;
	float fTmp;
	double P[3];

	for (iFeature = 0; iFeature < NodeArray.n; iFeature++)
	{
		pFeature = NodeArray.Element + iFeature;

		if (!pFeature->bEdge)
			continue;

		// N <- edge feature normal

		N = pFeature->N;

		// NParent <- parent surfel normal

		pSEdgePtr = pFeature->EdgeList.pFirst;

		iParentSurfel = RVLPCSEGMENT_GRAPH_GET_OPPOSITE_NODE(pSEdgePtr);

		pParentSurfel = NodeArray.Element + iParentSurfel;

		NParent = pParentSurfel->N;		

		// V <- unit(NParent x N)

		RVLCROSSPRODUCT3(NParent, N, V);

		RVLNORM3(V, fTmp);

		// P1 <- the first endpoint of the edge feature

		P1 = pFeature->P;

		// P2 <- P1 + pFeature->physicalSize * V

		RVLSCALE3VECTOR(V, pFeature->physicalSize, VTmp);

		RVLSUM3VECTORS(P1, VTmp, P2);

		// U <- DisplayData.edgeFeatureDepth * unit(V x N)

		RVLCROSSPRODUCT3(V, N, U);

		RVLNORM3(U, fTmp);

		RVLSCALE3VECTOR(U, DisplayData.edgeFeatureDepth, U);

		// P3 <- P1 + U

		RVLSUM3VECTORS(P1, U, P3);

		// P4 <- P2 + U

		RVLSUM3VECTORS(P2, U, P4);

		// Add P1, P2, P3 and P4 to pts

		RVLCOPY3VECTOR(P1, P);

		pts->InsertNextPoint(P);

		RVLCOPY3VECTOR(P2, P);

		pts->InsertNextPoint(P);

		RVLCOPY3VECTOR(P4, P);

		pts->InsertNextPoint(P);

		RVLCOPY3VECTOR(P3, P);

		pts->InsertNextPoint(P);

		// Create rectangle P1-P2-P3-P4.

		polyLine[iEdgeFeature] = vtkSmartPointer<vtkPolyLine>::New();

		polyLine[iEdgeFeature]->GetPointIds()->SetNumberOfIds(5);

		for (i = 0; i < 4; i++)
			polyLine[iEdgeFeature]->GetPointIds()->SetId(i, 4 * iEdgeFeature + i);

		polyLine[iEdgeFeature]->GetPointIds()->SetId(4, 4 * iEdgeFeature);

		// Add polyline to polyLines.

		polyLines->InsertNextCell(polyLine[iEdgeFeature]);

		// Assign color to polyline.

		//colors->InsertNextTupleValue(red);

		iEdgeFeature++;
	}

	// Add the points to the polydata container
	DisplayData.edgeFeaturesPolyData->SetPoints(pts);

	// Add the lines to the polydata container
	DisplayData.edgeFeaturesPolyData->SetLines(polyLines);

	// Color the lines.
	//DisplayData.edgeFeaturesPolyData->GetCellData()->SetScalars(colors);

	// Setup the visualization pipeline
	vtkSmartPointer<vtkPolyDataMapper> mapper =	vtkSmartPointer<vtkPolyDataMapper>::New();

	mapper->SetInputData(DisplayData.edgeFeaturesPolyData);

	DisplayData.edgeFeatures = vtkSmartPointer<vtkActor>::New();
	DisplayData.edgeFeatures->SetMapper(mapper);

	pVisualizer->renderer->AddActor(DisplayData.edgeFeatures);
}

void SurfelGraph::Save(
	int iSurfel,
	Mesh *pMesh,
	FILE *fpPoints,
	FILE *fpEdges)
{
	Surfel *pSurfel = NodeArray.Element + iSurfel;

	int iPt, iPt_;
	Point *pPt;
	MeshEdge *pEdge;
	MeshEdgePtr *pEdgePtr;

	QLIST::Index2 *pPtIdx = pSurfel->PtList.pFirst;

	while (pPtIdx)
	{
		iPt = pPtIdx->Idx;

		pPt = pMesh->NodeArray.Element + iPt;

		fprintf(fpPoints, "%d\t%f\t%f\t%f\t%d\n", iPt, pPt->P[0], pPt->P[1], pPt->P[2], 1);

		pEdgePtr = pPt->EdgeList.pFirst;

		while (pEdgePtr)
		{
			RVLPCSEGMENT_GRAPH_GET_NEIGHBOR(iPt, pEdgePtr, pEdge, iPt_);

			if (surfelMap[iPt_] == iSurfel)
				if (iPt < iPt_)
					fprintf(fpEdges, "%d\t%d\t%d\t1\n", pEdge->idx, iPt, iPt_);

			pEdgePtr = pEdgePtr->pNext;
		}

		pPtIdx = pPtIdx->pNext;
	}
}

void SurfelGraph::SaveSurfel(
	FILE *fp,
	int iSurfel)
{
	fwrite(&iSurfel, sizeof(int), 1, fp);
	
	Surfel *pSurfel = NodeArray.Element + iSurfel;

	fwrite(pSurfel->N, sizeof(float), 3, fp);
	fwrite(&(pSurfel->d), sizeof(float), 1, fp);
	fwrite(pSurfel->P, sizeof(float), 3, fp);
	fwrite(pSurfel->RGB, sizeof(int), 3, fp);
}

void SurfelGraph::LoadSurfel(
	FILE *fp,
	int iSurfel)
{
	fread(&iSurfel, sizeof(int), 1, fp);

	Surfel *pSurfel = NodeArray.Element + iSurfel;

	fread(pSurfel->N, sizeof(float), 3, fp);
	fread(&(pSurfel->d), sizeof(float), 1, fp);
	fread(pSurfel->P, sizeof(float), 3, fp);
	fread(pSurfel->RGB, sizeof(int), 3, fp);
}

void SurfelGraph::Save(
	FILE *fp,
	char *meshFileName,
	void *vpDetector)
{
	char header[] = "RVL::SurfelGraph 000";

	int headerLength = strlen(header);

	sprintf(header + headerLength - 3, "%03d", RVLSURFEL_VERSION_0);

	fwrite(header, sizeof(char), headerLength + 1, fp);

	fwrite(meshFileName, sizeof(char), strlen(meshFileName) + 1, fp);

	PlanarSurfelDetector *pDetector = (PlanarSurfelDetector *)vpDetector;

	pDetector->Save(fp);

	fwrite(&nMeshVertices, sizeof(int), 1, fp);
	fwrite(surfelMap, sizeof(int), nMeshVertices, fp);

	fwrite(&(NodeArray.n), sizeof(int), 1, fp);

	int iSurfel;

	for (iSurfel = 0; iSurfel < NodeArray.n; iSurfel++)
		if (NodeArray.Element[iSurfel].size > 0)
			SaveSurfel(fp, iSurfel);
}