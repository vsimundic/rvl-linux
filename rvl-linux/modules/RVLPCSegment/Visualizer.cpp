#include "RVLCore2.h"
#include "RVLVTK.h"
#include <vtkLine.h>
#include <vtkAxesActor.h>
#include <vtkParametricFunctionSource.h>
#include <vtkParametricEllipsoid.h>
#include <vtkOBJReader.h>
#include <vtkQuad.h>
#include "Util.h"
#include "Graph.h"
#ifdef RVLLINUX
#include <Eigen/Eigenvalues>
#else
#include <Eigen\Eigenvalues>
#endif
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

		//Coordinate system
		
		vtkSmartPointer<vtkAxesActor> axes =
			vtkSmartPointer<vtkAxesActor>::New();

		widget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
		widget->SetOutlineColor(0.9300, 0.5700, 0.1300);
		widget->SetOrientationMarker(axes);
		widget->SetInteractor(interactor);
		widget->SetViewport(0.0, 0.0, 0.3, 0.3);
		widget->SetEnabled(1);
		//widget->InteractiveOn();
		widget->InteractiveOff();

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

void Visualizer::SetMesh(vtkSmartPointer<vtkPolyData> pPolyData)
{
	map = vtkSmartPointer<vtkPolyDataMapper>::New();
	map->SetInputData(pPolyData);
	map->InterpolateScalarsBeforeMappingOff();

	//Actor
	actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(map);

	//Insert actor
	renderer->AddActor(actor);
}

void Visualizer::SetMesh(Mesh *pMesh)
{
	//vtkSmartPointer<vtkVertexGlyphFilter> vertexGlyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
	//vertexGlyphFilter->AddInputData(pMesh->pPolygonData);

	SetMesh(pMesh->pPolygonData);

	//Normals
	if (bNormals)
	{
		Normals(pMesh);

		if (!bNormalsVisible)
			normals->VisibilityOff();
	}
}

void Visualizer::SetMeshFromOBJ(char* fileName)
{
	vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
	reader->SetFileName(fileName);
	reader->Update();

	vtkSmartPointer<vtkPolyData> pPolyData = reader->GetOutput();

	SetMesh(pPolyData);
}

void Visualizer::Normals(
	Mesh *pMesh,
	float* NIn)
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
		N = (NIn ? NIn + 3 * iPt : pPt->N);

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

		colors->InsertNextTypedTuple(red);
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

void Visualizer::SetMouseMButtonDownCallback(
	void(*f)(vtkObject* caller, unsigned long eid, void* clientdata, void* calldata),
	void* clientData)
{
	//MiddleMouseButton callback
	mouseMButtonDownCallback = vtkSmartPointer<vtkCallbackCommand>::New();
	mouseMButtonDownCallback->SetCallback(f);
	mouseMButtonDownCallback->SetClientData(clientData);
	interactor->AddObserver(vtkCommand::MiddleButtonPressEvent, mouseMButtonDownCallback);
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

		rgbPointData->SetTypedTuple(iPt, Color);
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
	Figure *pFig,
	int *mapPixToPolyData)
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

	int u, v, iPt;
	char *pPix;

	QLIST::Index2 *pPtIdx = piPtList->pFirst;

	while (pPtIdx)
	{
		iPt = (mapPixToPolyData ? mapPixToPolyData[pPtIdx->Idx] : pPtIdx->Idx);

		if (b3D)
			rgbPointData->SetTypedTuple(iPt, Color);

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
			rgbPointData->SetTypedTuple(pPtIdx->Idx, Color);

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
			rgbPointData->SetTypedTuple(piPtArray->Element[i], Color);

		if (b2D)
			RVLVISUALIZER_SET_PIXEL_COLOR2(pPixArray, piPtArray->Element[i], width, widthStep, Color, u, v, pPix);
	}		
}

void Visualizer::AddMesh(
	vtkSmartPointer<vtkPolyData> &pd,
	double *color)
{
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(pd);
	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	if (color)
		actor->GetProperty()->SetColor(color);
	actor->SetMapper(mapper);

	renderer->AddActor(actor);
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

	colors->InsertNextTypedTuple(red);

	// y-axis

	RVLCOPYCOLMX3X3(R, 1, V);

	RVLSCALE3VECTOR(V, size, V);

	RVLSUM3VECTORS(P1, V, P2);

	pts->InsertNextPoint(P2);

	vtkSmartPointer<vtkLine> yAxis = vtkSmartPointer<vtkLine>::New();

	yAxis->GetPointIds()->SetId(0, iPt0);
	yAxis->GetPointIds()->SetId(1, iPt0 + 2);

	lines->InsertNextCell(yAxis);

	colors->InsertNextTypedTuple(green);

	// z-axis

	RVLCOPYCOLMX3X3(R, 2, V);

	RVLSCALE3VECTOR(V, size, V);

	RVLSUM3VECTORS(P1, V, P2);

	pts->InsertNextPoint(P2);

	vtkSmartPointer<vtkLine> zAxis = vtkSmartPointer<vtkLine>::New();

	zAxis->GetPointIds()->SetId(0, iPt0);
	zAxis->GetPointIds()->SetId(1, iPt0 + 3);

	lines->InsertNextCell(zAxis);

	colors->InsertNextTypedTuple(blue);
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

vtkSmartPointer<vtkActor> Visualizer::DisplayBox(
	double a,
	double b,
	double c,
	Pose3D *pPose,
	double red,
	double green,
	double blue,
	bool bWireFrame,
	float lineWidth)
{
#ifdef NEVER
	/// Old method.

	// Create a cube.
	vtkSmartPointer<vtkCubeSource> cubeSource =
		vtkSmartPointer<vtkCubeSource>::New();
	cubeSource->SetXLength(a);
	cubeSource->SetYLength(b);
	cubeSource->SetZLength(c);

	//Create transformation.
	double T[16];
	RVLHTRANSFMX(pPose->R, pPose->t, T);
	vtkSmartPointer<vtkTransform> pose = vtkSmartPointer<vtkTransform>::New();
	pose->SetMatrix(T);

	// Create a mapper and actor.
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(cubeSource->GetOutputPort());
	actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(red, green, blue);
	actor->SetUserTransform(pose);

	// Wireframe display.
	if(bWireFrame)
		actor->GetProperty()->SetRepresentationToWireframe();

	///
#endif

	Array<Point> vertices;
	Point vertexMem[8];
	vertices.Element = vertexMem;
	vertices.n = 8;
	float R[9] = {0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
	Pair<int, int> edgeMem[12];
	Array<Pair<int, int>> edges;
	edges.Element = edgeMem;
	edges.n = 12;
	int i, j;	
	Point *pPt, *pPt_;
	for (j = 0; j < 2; j++)
	{
		pPt = vertices.Element + 4 * j;
		RVLSET3VECTOR(pPt->P, 1.0, 1.0, -1.0 + 2.0 * (float)j);
		for (i = 1; i < 4; i++)
		{
			pPt_ = pPt + 1;
			RVLMULMX3X3VECT(R, pPt->P, pPt_->P);
			pPt = pPt_;
		}
		for (i = 0; i < 4; i++)
		{
			edges.Element[4 * j + i].a = 4 * j + i;
			edges.Element[4 * j + i].b = 4 * j + (i + 1) % 4;
		}
	}
	float s[3];
	RVLSET3VECTOR(s, a, b, c);
	RVLSCALE3VECTOR(s, 0.5f, s);
	float P[3];
	for (i = 0; i < 8; i++)
	{
		pPt = vertices.Element + i;
		RVLSCALE3VECTOR3(pPt->P, s, P);
		RVLTRANSF3(P, pPose->R, pPose->t, pPt->P);
	}
	for (i = 0; i < 4; i++)
	{
		edges.Element[8 + i].a = i;
		edges.Element[8 + i].b = i + 4;
	}

	uchar color[3];
	RVLSET3VECTOR(color, (uchar)red, (uchar)green, (uchar)blue);
	actor = DisplayLines(vertices, edges, color, lineWidth);	

	// Add actor to renderer.
	renderer->AddActor(actor);

	// Return actor.
	return actor;
}

vtkSmartPointer<vtkActor> Visualizer::DisplayCylinder(
	double r,
	double h,
	Pose3D *pPose,
	int resolution,
	double red,
	double green,
	double blue)
{
	// Create a cylinder
	vtkSmartPointer<vtkCylinderSource> cylinderSource = vtkSmartPointer<vtkCylinderSource>::New();
	//cylinderSource->SetCenter(0.0, 0.0, 0.0);
	cylinderSource->SetRadius(r);
	cylinderSource->SetHeight(h);
	cylinderSource->SetResolution(resolution);

	// Make z-os be the cylinder axis.
	float RotX[9];
	RVLROTX(0.0f, 1.0f, RotX);
	float R[9];
	RVLMXMUL3X3(pPose->R, RotX, R);

	// Create transformation.
	double T[16];
	RVLHTRANSFMX(R, pPose->t, T);
	vtkSmartPointer<vtkTransform> pose = vtkSmartPointer<vtkTransform>::New();
	pose->SetMatrix(T);

	// Create a mapper and actor
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(cylinderSource->GetOutputPort());
	actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(red, green, blue);
	actor->SetUserTransform(pose);

	// Add actor to renderer.
	renderer->AddActor(actor);

	return actor;
}

void Visualizer::DisplayEllipsoid(
	float *P,
	float *C,
	float r)
{
	float R[9];
	float var[3];

	DistributionFromCovMx(C, R, var);

	vtkSmartPointer<vtkParametricEllipsoid> pEllipsoid = vtkSmartPointer<vtkParametricEllipsoid>::New();
	pEllipsoid->SetXRadius(r * sqrt(var[0]));
	pEllipsoid->SetYRadius(r * sqrt(var[1]));
	pEllipsoid->SetZRadius(r * sqrt(var[2]));
	
	vtkSmartPointer<vtkParametricFunctionSource> pParametricFunctionSource = vtkSmartPointer<vtkParametricFunctionSource>::New();

	pParametricFunctionSource->SetParametricFunction(pEllipsoid);

	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	double T[16];
	RVLHTRANSFMX(R, P, T);
	transform->SetMatrix(T);

	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformFilter->SetInputConnection(pParametricFunctionSource->GetOutputPort()); //PLY model
	transformFilter->SetTransform(transform);
	transformFilter->Update();

	//Mapper
	map = vtkSmartPointer<vtkPolyDataMapper>::New();
	//map->SetInputConnection(pParametricFunctionSource->GetOutputPort());		// outside
	map->SetInputConnection(transformFilter->GetOutputPort());
	map->InterpolateScalarsBeforeMappingOff();

	//Actor
	actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(map);

	//Insert actor
	renderer->AddActor(actor);
}

vtkSmartPointer<vtkActor> Visualizer::DisplayLine(
	Point* vertex,
	uchar* color,
	float lineWidth)
{
	// Create the polydata where we will store all the geometric data
	vtkSmartPointer<vtkPolyData> linesPolyData =
		vtkSmartPointer<vtkPolyData>::New();

	// Create a vtkPoints container and store the points in it
	vtkSmartPointer<vtkPoints> pts =
		vtkSmartPointer<vtkPoints>::New();

	double P[3];
	RVLCOPY3VECTOR(vertex[0].P, P);
	pts->InsertNextPoint(P);
	RVLCOPY3VECTOR(vertex[1].P, P);
	pts->InsertNextPoint(P);

	// Add the points to the polydata container
	linesPolyData->SetPoints(pts);

	// Create lines.

	vtkSmartPointer<vtkCellArray> vtkLines =
		vtkSmartPointer<vtkCellArray>::New();

	vtkSmartPointer<vtkUnsignedCharArray> colors =
		vtkSmartPointer<vtkUnsignedCharArray>::New();

	colors->SetNumberOfComponents(3);

	vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();

	line->GetPointIds()->SetId(0, 0);
	line->GetPointIds()->SetId(1, 1);

	vtkLines->InsertNextCell(line);

	colors->InsertNextTypedTuple(color);

	// Add the lines to the polydata container
	linesPolyData->SetLines(vtkLines);

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
	vtkSmartPointer<vtkActor> lineActor = vtkSmartPointer<vtkActor>::New();
	lineActor->SetMapper(mapper);
	lineActor->GetProperty()->SetLineWidth(lineWidth);

	renderer->AddActor(lineActor);

	return lineActor;
}

vtkSmartPointer<vtkActor> Visualizer::DisplayLines(
	Array<Point> vertices,
	Array<Pair<int, int>> lines,
	uchar* color,
	float lineWidth)
{
	// Create the polydata where we will store all the geometric data
	vtkSmartPointer<vtkPolyData> linesPolyData =
		vtkSmartPointer<vtkPolyData>::New();

	// Create a vtkPoints container and store the points in it
	vtkSmartPointer<vtkPoints> pts =
		vtkSmartPointer<vtkPoints>::New();

	int iVertex;
	Point* pVertex;
	double P[3];

	for (iVertex = 0; iVertex < vertices.n; iVertex++)
	{
		pVertex = vertices.Element + iVertex;

		RVLCOPY3VECTOR(pVertex->P, P);

		pts->InsertNextPoint(P);
	}

	// Add the points to the polydata container
	linesPolyData->SetPoints(pts);

	// Create lines.

	vtkSmartPointer<vtkCellArray> vtkLines =
		vtkSmartPointer<vtkCellArray>::New();

	vtkSmartPointer<vtkUnsignedCharArray> colors =
		vtkSmartPointer<vtkUnsignedCharArray>::New();

	colors->SetNumberOfComponents(3);

	vtkSmartPointer<vtkLine>* line = new vtkSmartPointer<vtkLine>[lines.n];

	int iLine;

	for (iLine = 0; iLine < lines.n; iLine++)
	{
		line[iLine] = vtkSmartPointer<vtkLine>::New();

		line[iLine]->GetPointIds()->SetId(0, lines.Element[iLine].a);
		line[iLine]->GetPointIds()->SetId(1, lines.Element[iLine].b);

		vtkLines->InsertNextCell(line[iLine]);

		colors->InsertNextTypedTuple(color);
	}

	// Add the lines to the polydata container
	linesPolyData->SetLines(vtkLines);

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
	vtkSmartPointer<vtkActor> edges = vtkSmartPointer<vtkActor>::New();
	edges->SetMapper(mapper);
	edges->GetProperty()->SetLineWidth(lineWidth);

	renderer->AddActor(edges);

	delete[] line;

	return edges;
}

void Visualizer::DisplayReferenceFrames(
	Array<Pose3D> referenceFrames,
	double axesLength)
{
	// Create the polydata where we will store all the geometric data
	vtkSmartPointer<vtkPolyData> referenceFramesPolyData = vtkSmartPointer<vtkPolyData>::New();

	// Create a vtkPoints container and store the points in it
	vtkSmartPointer<vtkPoints> pts =
		vtkSmartPointer<vtkPoints>::New();

	// Create lines.
	vtkSmartPointer<vtkCellArray> lines =
		vtkSmartPointer<vtkCellArray>::New();

	// Create colors.
	vtkSmartPointer<vtkUnsignedCharArray> colors =
		vtkSmartPointer<vtkUnsignedCharArray>::New();

	colors->SetNumberOfComponents(3);

	int i;

	for (i = 0; i < referenceFrames.n; i++)
		AddReferenceFrame(pts, lines, colors, referenceFrames.Element[i].R, referenceFrames.Element[i].t, axesLength);

	// Add the points to the polydata container
	referenceFramesPolyData->SetPoints(pts);

	// Add the lines to the polydata container
	referenceFramesPolyData->SetLines(lines);

	// Color the lines.
	referenceFramesPolyData->GetCellData()->SetScalars(colors);

	// Setup the visualization pipeline
	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();

	mapper->SetInputData(referenceFramesPolyData);

	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);

	renderer->AddActor(actor);
}

void Visualizer::DisplayReferenceFrame(
	Pose3D *pReferenceFrame,
	double axesLength)
{
	// Create the polydata where we will store all the geometric data
	vtkSmartPointer<vtkPolyData> referenceFramesPolyData = vtkSmartPointer<vtkPolyData>::New();

	// Create a vtkPoints container and store the points in it
	vtkSmartPointer<vtkPoints> pts =
		vtkSmartPointer<vtkPoints>::New();

	// Create lines.
	vtkSmartPointer<vtkCellArray> lines =
		vtkSmartPointer<vtkCellArray>::New();

	// Create colors.
	vtkSmartPointer<vtkUnsignedCharArray> colors =
		vtkSmartPointer<vtkUnsignedCharArray>::New();

	colors->SetNumberOfComponents(3);

	AddReferenceFrame(pts, lines, colors, pReferenceFrame->R, pReferenceFrame->t, axesLength);

	// Add the points to the polydata container
	referenceFramesPolyData->SetPoints(pts);

	// Add the lines to the polydata container
	referenceFramesPolyData->SetLines(lines);

	// Color the lines.
	referenceFramesPolyData->GetCellData()->SetScalars(colors);

	// Setup the visualization pipeline
	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();

	mapper->SetInputData(referenceFramesPolyData);

	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);

	renderer->AddActor(actor);
}

vtkSmartPointer<vtkActor2D> Visualizer::DisplayLabels(vtkSmartPointer<vtkPolyData> ptsPolyData)
{
	vtkSmartPointer<vtkLabeledDataMapper> labelMapper = vtkSmartPointer<vtkLabeledDataMapper>::New();
	labelMapper->SetInputData(ptsPolyData);
	vtkSmartPointer<vtkActor2D> labelActor = vtkSmartPointer<vtkActor2D>::New();
	labelActor->SetMapper(labelMapper);
	renderer->AddActor2D(labelActor);

	return labelActor;
}

vtkSmartPointer<vtkActor2D> Visualizer::DisplayLabels(Array<Point> points)
{
	vtkSmartPointer<vtkPolyData> ptsPolyData = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPoints> points_ = vtkSmartPointer<vtkPoints>::New();
	int iPt;
	float* P;
	for (iPt = 0; iPt < points.n; iPt++)
	{
		P = points.Element[iPt].P;
		points_->InsertNextPoint(P);
	}
	ptsPolyData->SetPoints(points_);
	return DisplayLabels(ptsPolyData);
}

vtkSmartPointer<vtkActor> Visualizer::DisplayVoxels(
	Array3D<uchar> voxels,
	uchar* faceColor,
	float opacity,
	uchar* edgeColor,
	bool bHideEdges)
{
	uchar defaultFaceColor[3] = {255, 255, 255};
	uchar defaultEdgeColor[3] = { 0, 0, 0 };
	uchar* faceColor_ = (faceColor ? faceColor : defaultFaceColor);
	uchar* edgeColor_ = (edgeColor ? edgeColor : defaultEdgeColor);

	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkCellArray> faces = vtkSmartPointer<vtkCellArray>::New();

	int nVoxels = voxels.a * voxels.b * voxels.c;
	uchar* voxel = voxels.Element;
	int i, x, y, z, dim, dim_, dim__, stride, u, v, normalDirection, iTmp;
	int iVox = 0;
	int iPt = 0;
	int coordinate[3], coordinate_[3];
	double P[3];
	unsigned char voxVal, voxVal_;
	vtkSmartPointer<vtkQuad> face;
	int n[3];
	RVLSET3VECTOR(n, voxels.a, voxels.b, voxels.c);
	for (z = -1; z < voxels.c; z++)
		for (y = -1; y < voxels.b; y++)
			for (x = -1; x < voxels.a; x++)
			{
				iVox = RVL3DARRAY_ELEMENT_INDEX(voxels, x, y, z);
				RVLSET3VECTOR(coordinate, x, y, z);
				for (dim = 0; dim < 3; dim++)
					if (coordinate[dim] < 0)
						break;
				voxVal = (dim >= 3 ? voxel[iVox] : 0);
				stride = 1;
				RVLCOPY3VECTOR(coordinate, coordinate_);
				for (dim = 0; dim < 3; dim++)
				{
					coordinate_[dim]++;
					for (dim_ = 0; dim_ < 3; dim_++)
						if (coordinate_[dim_] < 0 || coordinate_[dim_] >= n[dim_])
							break;
					voxVal_ = (dim_ >= 3 ? voxel[iVox + stride] : 0);
					if (voxVal != voxVal_)
					{
						face = vtkSmartPointer<vtkQuad>::New();
						normalDirection = (voxVal == 1 ? 1 : -1);
						P[dim] = (double)(coordinate[dim] + 1);
						dim_ = (dim + 1) % 3;
						dim__ = (dim_ + 1) % 3;
						u = -1;
						v = -1;
						for (i = 0; i < 4; i++)
						{
							P[dim_] = (double)coordinate[dim_] + (double)((u + 1) / 2);
							P[dim__] = (double)coordinate[dim__] + (double)((v + 1) / 2);
							points->InsertNextPoint(P[0], P[1], P[2]);
							iTmp = u;
							u = normalDirection * v;
							v = -normalDirection * iTmp;
							face->GetPointIds()->SetId(3 - i, iPt++);
						}
						faces->InsertNextCell(face);
					}
					coordinate_[dim]--;
					stride *= n[dim];
				}
			}
	vtkSmartPointer<vtkPolyData> mesh = vtkSmartPointer<vtkPolyData>::New();
	mesh->SetPoints(points);
	mesh->SetPolys(faces);
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(mesh);
	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	//actor->GetProperty()->SetAmbientColor(faceColor_[0], faceColor_[1], faceColor_[2]);
	actor->GetProperty()->SetDiffuseColor((double)faceColor_[0] / 255.0, (double)faceColor_[1] / 255, (double)faceColor_[2] / 255);
	actor->GetProperty()->SetRepresentationToSurface();
	actor->GetProperty()->SetOpacity(opacity);
	if (!bHideEdges)
	{
		actor->GetProperty()->EdgeVisibilityOn();
		actor->GetProperty()->SetEdgeColor(edgeColor_[0], edgeColor_[1], edgeColor_[2]);
	}
	actor->GetProperty()->BackfaceCullingOn();

	return actor;
}

void Visualizer::DisplayOrganizedOrientedPC(
	Array2D<OrientedPoint> PC,
	IplImage *rgbImage,
	uchar* pointColorIn,
	float normalLen,	
	uchar* normalColorIn)
{
	uchar gray[] = {128, 128, 128};
	char pointColor[3];
	if(pointColorIn)
	{
		RVLCOPY3VECTOR(pointColorIn, pointColor);
	}
	else
	{
		RVLCOPY3VECTOR(gray, pointColor);
	}
	uchar white[] = { 255, 255, 255 };
	uchar* normalColor = (normalColorIn ? normalColorIn : white);
	int nPix = PC.w * PC.h;
	int iPix;
	OrientedPoint* pPt;
	int u, v;
	Array<Point> visPts;
	visPts.Element = new Point[nPix];
	Point* pVisPt = visPts.Element;
	visPts.n = 0;
	uchar* RGB = new uchar[3 * nPix];
	uchar* rgbTgt;
	char* bgrSrc;
	for (iPix = 0; iPix < nPix; iPix++)
	{
		pPt = PC.Element + iPix;
		if (pPt->P[2] <= 0.0f)
			continue;
		if (RVLDOTPRODUCT3(pPt->N, pPt->N) < 0.5f)
			continue;
		pVisPt = visPts.Element + visPts.n;
		RVLCOPY3VECTOR(pPt->P, pVisPt->P);
		RVLCOPY3VECTOR(pPt->N, pVisPt->N);
		rgbTgt = RGB + 3 * visPts.n;
		if (rgbImage)
			bgrSrc = rgbImage->imageData + 3 * iPix;
		else
			bgrSrc = pointColor;
		RVLSET3VECTOR(rgbTgt, bgrSrc[2], bgrSrc[1], bgrSrc[0]);
		visPts.n++;
	}
	Array<Point> visNormalPts;
	int subsamplingStep = 2;
	visNormalPts.Element = new Point[nPix / subsamplingStep];
	visNormalPts.n = 0;
	Array<Pair<int, int>> visNormals;
	visNormals.Element = new Pair<int, int>[nPix / (subsamplingStep * subsamplingStep)];
	visNormals.n = 0;
	Pair<int, int>* pVisNormal;
	float V3Tmp[3];
	for (v = 0; v < PC.h; v += 2)
		for (u = 0; u < PC.w; u += 2)
		{
			iPix = u + v * PC.w;
			pPt = PC.Element + iPix;
			if (pPt->P[2] <= 0.0f)
				continue;
			if (RVLDOTPRODUCT3(pPt->N, pPt->N) < 0.5f)
				continue;
			pVisPt = visNormalPts.Element + visNormalPts.n;
			RVLCOPY3VECTOR(pPt->P, pVisPt->P);
			pVisPt++;
			RVLSCALE3VECTOR(pPt->N, normalLen, V3Tmp);
			RVLSUM3VECTORS(pPt->P, V3Tmp, pVisPt->P);
			pVisNormal = visNormals.Element + visNormals.n;
			pVisNormal->a = visNormalPts.n;
			visNormalPts.n++;
			pVisNormal->b = visNormalPts.n;
			visNormalPts.n++;
			visNormals.n++;
		}
	DisplayPointSet<float, Point>(visPts, RGB, 4, true);
	DisplayLines(visNormalPts, visNormals, normalColor);
	delete[] visPts.Element;
	delete[] RGB;
	delete[] visNormalPts.Element;
	delete[] visNormals.Element;
}
