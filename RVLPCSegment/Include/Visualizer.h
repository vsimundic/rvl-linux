#pragma once

namespace RVL
{
	class Visualizer
	{
	public:
		Visualizer();
		virtual ~Visualizer();
		void Create();
		void SetWindowSize(int width, int height);
		void SetBackgroundColor(double r, double g, double b);
		void SetMesh(Mesh *pMesh);
		void SetKeyPressCallback(
			void(*f)(vtkObject *caller, unsigned long eid, void *clientdata, void *calldata),
			void *clientData);
		void Visualizer::SetMouseRButtonDownCallback(
			void(*f)(vtkObject *caller, unsigned long eid, void *clientdata, void *calldata),
			void *clientData);
		void SetText(char * textIn);
		void Run();
		void PaintPoint(
			int iPt,
			vtkSmartPointer<vtkPolyData> &pd,
			unsigned char *Color);
		void PaintPointSet(
			QList<QLIST::Index> *piPtList,
			vtkSmartPointer<vtkPolyData> &pd,
			unsigned char *Color);
		void PaintPointSet(
			Array<int> *piPtArray,
			vtkSmartPointer<vtkPolyData> &pd,
			unsigned char *Color);

	public:
		vtkSmartPointer<vtkRenderer> renderer;
		vtkSmartPointer<vtkRenderWindow> window;
		vtkSmartPointer<vtkRenderWindowInteractor> interactor;
		vtkSmartPointer<vtkInteractorStyleTrackballCamera> style;
		vtkSmartPointer<vtkPolyDataMapper> map;
		vtkSmartPointer<vtkActor> actor;
		vtkSmartPointer<vtkPointPicker> pointPicker;
		vtkSmartPointer<vtkCornerAnnotation> text;
		vtkSmartPointer<vtkCallbackCommand> keypressCallback;
		vtkSmartPointer<vtkCallbackCommand> mouseRButtonDownCallback;		
	};
}

