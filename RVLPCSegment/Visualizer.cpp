//#include "stdafx.h"
#include "RVLVTK.h"
#include <vtkLine.h>
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

using namespace RVL;

Visualizer::Visualizer()
{
	normalLength = 1.0;
	bNormals = false;
	bNormalsVisible = false;
	b3D = true;
	b2D = false;
}


Visualizer::~Visualizer()
{
	int iFig;
	Figure *pFig;

	for (iFig = 0; iFig < figures.size(); iFig++)
	{
		pFig = figures.at(iFig);

		delete pFig;
	}
}


void Visualizer::Create()
{
	// Initialize VTK.

	if (b3D)
	{
		renderer = vtkSmartPointer<vtkRenderer>::New();
		window = vtkSmartPointer<vtkRenderWindow>::New();
		interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
		window->AddRenderer(renderer);
		window->SetSize(800, 600);
		interactor->SetRenderWindow(window);
		style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
		interactor->SetInteractorStyle(style);
		renderer->SetBackground(0.5294, 0.8078, 0.9803);

		////Mapper
		//map = vtkSmartPointer<vtkPolyDataMapper>::New();
		//map->SetInputData(pd);		// outside
		////map->SetInputConnection(polyDataNormals->GetOutputPort());
		//map->InterpolateScalarsBeforeMappingOff();

		////Actor
		//actor = vtkSmartPointer<vtkActor>::New();
		//actor->SetMapper(map);

		////Insert actor
		//renderer->AddActor(actor);

		//Point picker
		pointPicker = vtkSmartPointer<vtkPointPicker>::New();
		interactor->SetPicker(pointPicker);

		////Text
		text = vtkSmartPointer<vtkCornerAnnotation>::New();
		text->SetLinearFontScaleFactor(2);
		text->SetNonlinearFontScaleFactor(1);
		text->SetMaximumFontSize(15);
		//text->SetText(0, "Text...");
		text->GetTextProperty()->SetColor(1, 1, 0);
		renderer->AddViewProp(text);

		////Keypress callback
		//keypressCallback = vtkSmartPointer<vtkCallbackCommand>::New();
		//keypressCallback->SetCallback(KeyPressCallback);
		//keypressCallback->SetClientData(&DisplayData);
		//interactor->AddObserver(vtkCommand::KeyPressEvent, keypressCallback);

		////RightMouseButton callback
		//mouseRButtonDownCallback = vtkSmartPointer<vtkCallbackCommand>::New();
		//mouseRButtonDownCallback->SetCallback(MouseRButtonDown);
		//mouseRButtonDownCallback->SetClientData(&DisplayData);
		//interactor->AddObserver(vtkCommand::RightButtonPressEvent, mouseRButtonDownCallback);
	}
}


void Visualizer::SetWindowSize(int width, int height)
{
	window->SetSize(width, height);
}


void Visualizer::SetBackgroundColor(double r, double g, double b)
{
	renderer->SetBackground(r, g, b);
}


void Visualizer::SetMesh(Mesh *pMesh)
{
	//Mapper
	map = vtkSmartPointer<vtkPolyDataMapper>::New();
	map->SetInputData(pMesh->pPolygonData);		// outside
	//map->SetInputConnection(polyDataNormals->GetOutputPort());
	map->InterpolateScalarsBeforeMappingOff();

	//Actor
	actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(map);

	//Insert actor
	renderer->AddActor(actor);

	//Normals
	if (bNormals)
	{
		Normals(pMesh);

		if (!bNormalsVisible)
			normals->VisibilityOff();
	}
}

void Visualizer::Normals(Mesh *pMesh)
{
	// Create the polydata where we will store all the geometric data
	vtkSmartPointer<vtkPolyData> linesPolyData =
		vtkSmartPointer<vtkPolyData>::New();

	// Create a vtkPoints container and store the points in it
	vtkSmartPointer<vtkPoints> pts =
		vtkSmartPointer<vtkPoints>::New();

	int iLine = 0;

	double P0[3], P1[3], V[3];
	int iPt;
	Point *pPt;
	float *P, *N;

	for (iPt = 0; iPt < pMesh->NodeArray.n; iPt++)
	{
		pPt = pMesh->NodeArray.Element + iPt;

		P = pPt->P;
		N = pPt->N;

		RVLCOPY3VECTOR(P, P0);

		pts->InsertNextPoint(P0);

		RVLSCALE3VECTOR(N, normalLength, V);

		RVLSUM3VECTORS(P0, V, P1);

		pts->InsertNextPoint(P1);

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
	normals = vtkSmartPointer<vtkActor>::New();
	normals->SetMapper(mapper);

	renderer->AddActor(normals);

	delete[] line;
}

void Visualizer::SetKeyPressCallback(
	void(*f)(vtkObject *caller, unsigned long eid, void *clientdata, void *calldata),
	void *clientData)
{
	//Keypress callback
	keypressCallback = vtkSmartPointer<vtkCallbackCommand>::New();
	keypressCallback->SetCallback(f);
	keypressCallback->SetClientData(clientData);
	interactor->AddObserver(vtkCommand::KeyPressEvent, keypressCallback);
}

void Visualizer::SetMouseRButtonDownCallback(
	void(*f)(vtkObject *caller, unsigned long eid, void *clientdata, void *calldata),
	void *clientData)
{
	//RightMouseButton callback
	mouseRButtonDownCallback = vtkSmartPointer<vtkCallbackCommand>::New();
	mouseRButtonDownCallback->SetCallback(f);
	mouseRButtonDownCallback->SetClientData(clientData);
	interactor->AddObserver(vtkCommand::RightButtonPressEvent, mouseRButtonDownCallback);
}

void Visualizer::SetText(char * textIn)
{
	//Text
	text = vtkSmartPointer<vtkCornerAnnotation>::New();
	text->SetLinearFontScaleFactor(2);
	text->SetNonlinearFontScaleFactor(1);
	text->SetMaximumFontSize(15);
	text->SetText(0, textIn);
	text->GetTextProperty()->SetColor(1, 1, 0);
	renderer->AddViewProp(text);
}

void Visualizer::Run()
{
	if (b3D)
	{
		//Rendering

		renderer->ResetCamera();
		window->Render();

		//Start interactor //Need a way to stop it!
		window->GetInteractor()->Start();
	}

	if (b2D)
	{
		int iFig;

		for (iFig = 0; iFig < figures.size(); iFig++)
			ShowFigure(figures.at(iFig));

		cv::waitKey();
	}
}

void Visualizer::PaintPoint(
	int iPt,
	vtkSmartPointer<vtkPolyData> &pd,
	unsigned char *Color,
	Figure *pFig)
{
	if (b3D)
	{
		vtkSmartPointer<vtkUnsignedCharArray> rgbPointData = rgbPointData->SafeDownCast(pd->GetPointData()->GetArray("RGB"));

		rgbPointData->SetTupleValue(iPt, Color);
	}

	if (b2D)
	{
		int width = pFig->pImage->width;

		int widthStep = pFig->pImage->widthStep;

		char *pPixArray = pFig->pImage->imageData;

		int u, v;
		char *pPix;

		RVLVISUALIZER_SET_PIXEL_COLOR2(pPixArray, iPt, width, widthStep, Color, u, v, pPix);
	}
}

void Visualizer::PaintPointSet(
	QList<QLIST::Index2> *piPtList,
	vtkSmartPointer<vtkPolyData> &pd,
	unsigned char *Color,
	Figure *pFig)
{
	vtkSmartPointer<vtkUnsignedCharArray> rgbPointData;

	if (b3D)
		rgbPointData = rgbPointData->SafeDownCast(pd->GetPointData()->GetArray("RGB"));

	int widthStep;
	char *pPixArray;
	int width;
	
	if (b2D)
	{
		width = pFig->pImage->width;

		widthStep = pFig->pImage->widthStep;

		pPixArray = pFig->pImage->imageData;
	}

	int u, v;
	char *pPix;

	QLIST::Index2 *pPtIdx = piPtList->pFirst;

	while (pPtIdx)
	{
		if (b3D)
			rgbPointData->SetTupleValue(pPtIdx->Idx, Color);

		if (b2D)
			RVLVISUALIZER_SET_PIXEL_COLOR2(pPixArray, pPtIdx->Idx, width, widthStep, Color, u, v, pPix);

		pPtIdx = pPtIdx->pNext;
	}
}

void Visualizer::PaintPointSet(
	QList<QLIST::Index> *piPtList,
	vtkSmartPointer<vtkPolyData> &pd,
	unsigned char *Color,
	Figure *pFig)
{
	vtkSmartPointer<vtkUnsignedCharArray> rgbPointData;

	if (b3D)
		rgbPointData = rgbPointData->SafeDownCast(pd->GetPointData()->GetArray("RGB"));

	int widthStep;
	char *pPixArray;
	int width;

	if (b2D)
	{
		width = pFig->pImage->width;

		widthStep = pFig->pImage->widthStep;

		pPixArray = pFig->pImage->imageData;
	}

	int u, v;
	char *pPix;

	QLIST::Index *pPtIdx = piPtList->pFirst;

	while (pPtIdx)
	{
		if (b3D)
			rgbPointData->SetTupleValue(pPtIdx->Idx, Color);

		if (b2D)
			RVLVISUALIZER_SET_PIXEL_COLOR2(pPixArray, pPtIdx->Idx, width, widthStep, Color, u, v, pPix);

		pPtIdx = pPtIdx->pNext;
	}
}

void Visualizer::PaintPointSet(
	Array<int> *piPtArray,
	vtkSmartPointer<vtkPolyData> &pd,
	unsigned char *Color,
	Figure *pFig)
{
	vtkSmartPointer<vtkUnsignedCharArray> rgbPointData;

	if (b3D)
		rgbPointData = rgbPointData->SafeDownCast(pd->GetPointData()->GetArray("RGB"));

	int widthStep;
	char *pPixArray;
	int width;

	if (b2D)
	{
		width = pFig->pImage->width;

		widthStep = pFig->pImage->widthStep;

		pPixArray = pFig->pImage->imageData;
	}

	int i;
	int u, v;
	char *pPix;

	for (i = 0; i < piPtArray->n; i++)
	{
		if (b3D)
			rgbPointData->SetTupleValue(piPtArray->Element[i], Color);

		if (b2D)
			RVLVISUALIZER_SET_PIXEL_COLOR2(pPixArray, piPtArray->Element[i], width, widthStep, Color, u, v, pPix);
	}		
}

void Visualizer::AddReferenceFrame(
	vtkSmartPointer<vtkPoints> &pts,
	vtkSmartPointer<vtkCellArray> &lines,
	vtkSmartPointer<vtkUnsignedCharArray> &colors,
	float *R,
	float *t,
	double size)
{
	double V[3], P1[3], P2[3];

	int iPt0 = pts->GetNumberOfPoints();

	// colors

	unsigned char red[3] = { 255, 0, 0 };
	unsigned char green[3] = { 0, 255, 0 };
	unsigned char blue[3] = { 0, 0, 255 };

	// origin

	RVLCOPY3VECTOR(t, P1);

	pts->InsertNextPoint(P1);

	// x-axis

	RVLCOPYCOLMX3X3(R, 0, V);

	RVLSCALE3VECTOR(V, size, V);

	RVLSUM3VECTORS(P1, V, P2);

	pts->InsertNextPoint(P2);

	vtkSmartPointer<vtkLine> xAxis = vtkSmartPointer<vtkLine>::New();

	xAxis->GetPointIds()->SetId(0, iPt0);
	xAxis->GetPointIds()->SetId(1, iPt0 + 1);

	lines->InsertNextCell(xAxis);

	colors->InsertNextTupleValue(red);

	// y-axis

	RVLCOPYCOLMX3X3(R, 1, V);

	RVLSCALE3VECTOR(V, size, V);

	RVLSUM3VECTORS(P1, V, P2);

	pts->InsertNextPoint(P2);

	vtkSmartPointer<vtkLine> yAxis = vtkSmartPointer<vtkLine>::New();

	yAxis->GetPointIds()->SetId(0, iPt0);
	yAxis->GetPointIds()->SetId(1, iPt0 + 2);

	lines->InsertNextCell(yAxis);

	colors->InsertNextTupleValue(green);

	// z-axis

	RVLCOPYCOLMX3X3(R, 2, V);

	RVLSCALE3VECTOR(V, size, V);

	RVLSUM3VECTORS(P1, V, P2);

	pts->InsertNextPoint(P2);

	vtkSmartPointer<vtkLine> zAxis = vtkSmartPointer<vtkLine>::New();

	zAxis->GetPointIds()->SetId(0, iPt0);
	zAxis->GetPointIds()->SetId(1, iPt0 + 3);

	lines->InsertNextCell(zAxis);

	colors->InsertNextTupleValue(blue);
}

Figure *Visualizer::OpenFigure(
	char *ImageName,
	int memSize)
{
	Figure *pFig;

	int iFig;

	for (iFig = 0; iFig < figures.size(); iFig++)
	{
		pFig = figures.at(iFig);

		if (strcmp(ImageName, pFig->name) == 0)
			return pFig;
	}

	pFig = new Figure;

	pFig->Create(memSize);

	figures.push_back(pFig);

	pFig->vpVisualizer = this;

	pFig->name = RVLCreateString(ImageName);

	return pFig;
}

void Visualizer::ShowFigure(char *imageName)
{
	Figure *pFig = OpenFigure(imageName);

	ShowFigure(pFig);
}

void Visualizer::ShowFigure(Figure *pFig)
{
	cvShowImage(pFig->name, pFig->pImage);
}
