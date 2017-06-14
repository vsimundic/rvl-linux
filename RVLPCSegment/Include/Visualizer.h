#pragma once

#include "Figure.h"

#define RVLVISUALIZER_SET_PIXEL_COLOR(pPixArray, u, v, widthStep, color, pPix)\
{\
	pPix = pPixArray + 3 * u + v * widthStep;\
	*(pPix++) = color[0];\
	*(pPix++) = color[1];\
	*pPix = color[2];\
}

#define RVLVISUALIZER_SET_PIXEL_COLOR2(pPixArray, iPix, width, widthStep, color, u, v, pPix)\
{\
	u = iPix % width;\
	v = iPix / width;\
	RVLVISUALIZER_SET_PIXEL_COLOR(pPixArray, u, v, widthStep, color, pPix);\
}

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
		void Normals(Mesh *pMesh);
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
			unsigned char *Color,
			Figure *pFig = NULL);
		void PaintPointSet(
			QList<QLIST::Index2> *piPtList,
			vtkSmartPointer<vtkPolyData> &pd,
			unsigned char *Color,
			Figure *pFig = NULL);
		void PaintPointSet(
			QList<QLIST::Index> *piPtList,
			vtkSmartPointer<vtkPolyData> &pd,
			unsigned char *Color,
			Figure *pFig = NULL);
		void PaintPointSet(
			Array<int> *piPtArray,
			vtkSmartPointer<vtkPolyData> &pd,
			unsigned char *Color,
			Figure *pFig = NULL);
		void AddReferenceFrame(
			vtkSmartPointer<vtkPoints> &pts,
			vtkSmartPointer<vtkCellArray> &lines,
			vtkSmartPointer<vtkUnsignedCharArray> &colors,
			float *R,
			float *t,
			double size);
		Figure *OpenFigure(
			char *ImageName,
			int memSize = 5000000);
		void ShowFigure(char *imageName);
		void ShowFigure(Figure *pFig);

	public:
		CRVLMem *pMem;
		vtkSmartPointer<vtkRenderer> renderer;
		vtkSmartPointer<vtkRenderWindow> window;
		vtkSmartPointer<vtkRenderWindowInteractor> interactor;
		vtkSmartPointer<vtkInteractorStyleTrackballCamera> style;
		vtkSmartPointer<vtkPolyDataMapper> map;
		vtkSmartPointer<vtkActor> actor;
		vtkSmartPointer<vtkActor> normals;
		vtkSmartPointer<vtkPointPicker> pointPicker;
		vtkSmartPointer<vtkCornerAnnotation> text;
		vtkSmartPointer<vtkCallbackCommand> keypressCallback;
		vtkSmartPointer<vtkCallbackCommand> mouseRButtonDownCallback;	
		double normalLength;
		bool bNormals;
		bool bNormalsVisible;
		bool b3D;
		bool b2D;
		std::vector<Figure *> figures;
	};
}

