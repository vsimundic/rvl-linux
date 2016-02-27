//#include "stdafx.h"
#include "RVLVTK.h"
#include "RVLCore2.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"

using namespace RVL;

Visualizer::Visualizer()
{

}


Visualizer::~Visualizer()
{
}


void Visualizer::Create()
{
	// Initialize VTK.
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
	//Rendering

	renderer->ResetCamera();
	window->Render();

	//Start interactor //Need a way to stop it!
	window->GetInteractor()->Start();
}

void Visualizer::PaintPoint(
	int iPt,
	vtkSmartPointer<vtkPolyData> &pd,
	unsigned char *Color)
{
	vtkSmartPointer<vtkUnsignedCharArray> rgbPointData = rgbPointData->SafeDownCast(pd->GetPointData()->GetArray("RGB"));

	rgbPointData->SetTupleValue(iPt, Color);
}

void Visualizer::PaintPointSet(
	QList<QLIST::Index> *piPtList,
	vtkSmartPointer<vtkPolyData> &pd,
	unsigned char *Color)
{
	vtkSmartPointer<vtkUnsignedCharArray> rgbPointData;

	rgbPointData = rgbPointData->SafeDownCast(pd->GetPointData()->GetArray("RGB"));

	QLIST::Index *pPtIdx = piPtList->pFirst;

	while (pPtIdx)
	{
		rgbPointData->SetTupleValue(pPtIdx->Idx, Color);

		pPtIdx = pPtIdx->pNext;
	}
}

void Visualizer::PaintPointSet(
	Array<int> *piPtArray,
	vtkSmartPointer<vtkPolyData> &pd,
	unsigned char *Color)
{
	vtkSmartPointer<vtkUnsignedCharArray> rgbPointData;

	rgbPointData = rgbPointData->SafeDownCast(pd->GetPointData()->GetArray("RGB"));

	int i;

	for (i = 0; i < piPtArray->n; i++)
		rgbPointData->SetTupleValue(piPtArray->Element[i], Color);
}