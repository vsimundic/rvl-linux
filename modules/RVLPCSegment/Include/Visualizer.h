#pragma once

#include "Figure.h"

#define RVLCOLORS                    \
	uchar black[] = {0, 0, 0};       \
	uchar red[] = {255, 0, 0};       \
	uchar green[] = {0, 255, 0};     \
	uchar blue[] = {0, 0, 255};      \
	uchar yellow[] = {255, 255, 0};  \
	uchar cyan[] = {0, 255, 255};    \
	uchar magenta[] = {255, 0, 255}; \
	uchar white[] = {255, 255, 255}; \
	uchar darkGreen[] = {0, 128, 0};

#define RVLVISUALIZER_SET_PIXEL_COLOR(pPixArray, u, v, widthStep, color, pPix) \
	{                                                                          \
		pPix = pPixArray + 3 * u + v * widthStep;                              \
		*(pPix++) = color[0];                                                  \
		*(pPix++) = color[1];                                                  \
		*pPix = color[2];                                                      \
	}

#define RVLVISUALIZER_SET_PIXEL_COLOR2(pPixArray, iPix, width, widthStep, color, u, v, pPix) \
	{                                                                                        \
		u = iPix % width;                                                                    \
		v = iPix / width;                                                                    \
		RVLVISUALIZER_SET_PIXEL_COLOR(pPixArray, u, v, widthStep, color, pPix);              \
	}

#define RVLVISUALIZER_LINES_INIT(pts, lines, nLines) \
	Array<Point> pts;                                \
	pts.Element = new Point[2 * nLines];             \
	pts.n = 2 * nLines;                              \
	Array<Pair<int, int>> lines;                     \
	lines.Element = new Pair<int, int>[nLines];      \
	lines.n = nLines;

#define RVLVISUALIZER_LINES_FREE(pts, lines) \
	delete[] pts.Element;                    \
	delete[] lines.Element;

namespace RVL
{
	class PointPickCallback : public vtkCommand
	{
	public:
		static PointPickCallback *New()
		{
			return new PointPickCallback;
		}

		void SetPolyData(vtkSmartPointer<vtkPolyData> data)
		{
			this->polyData = data;
		}

		void SetData(void *pDataIn)
		{
			this->pData = pDataIn;
		}

		void SetUserCallback(std::function<void(vtkIdType, double *, void *callData)> func)
		{
			this->UserCallback = func;
		}

		void Execute(vtkObject *caller, unsigned long eventId, void *callData) override
		{
			vtkRenderWindowInteractor *interactor = static_cast<vtkRenderWindowInteractor *>(caller);
			vtkCellPicker *picker = vtkCellPicker::SafeDownCast(interactor->GetPicker());

			if (!picker)
			{
				std::cerr << "No valid picker set.\n";
				return;
			}

			int eventPos[2];
			interactor->GetEventPosition(eventPos);

			vtkRenderer *renderer = interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer();
			picker->Pick(eventPos[0], eventPos[1], 0, renderer); // screen coordinates
			// std::cout << "Manual pick cell ID: " << picker->GetCellId() << std::endl;

			if (picker->GetCellId() >= 0)
			{
				double pickedPos[3];
				picker->GetPickPosition(pickedPos);

				vtkIdType closestPointId = polyData->FindPoint(pickedPos);
				double closestPoint[3];
				polyData->GetPoint(closestPointId, closestPoint);

				std::cout << "Picked cell ID: " << picker->GetCellId() << "\n";
				std::cout << "Closest point ID: " << closestPointId << "\n";
				std::cout << "Coords: (" << closestPoint[0] << ", "
						  << closestPoint[1] << ", " << closestPoint[2] << ")\n";

				if (UserCallback)
					UserCallback(closestPointId, closestPoint, pData);
			}
			else
			{
				std::cout << "No cell picked.\n";
			}
		}

	private:
		vtkSmartPointer<vtkPolyData> polyData;
		void *pData;
		std::function<void(vtkIdType, double *, void *callData)> UserCallback;
	};

	void PointPickerUserCallbackDemo(vtkIdType closestPointId, double *closestPoint, void *callData);
	void VTKPointPickerDemo();

	class Visualizer
	{
	public:
		Visualizer();
		virtual ~Visualizer();
		void Create();
		void SetWindowSize(int width, int height);
		void SetBackgroundColor(double r, double g, double b);
		void SetMesh(Mesh *pMesh);
		void SetMesh(vtkSmartPointer<vtkPolyData> pPolyData);
		void SetMeshFromOBJ(char *fileName);
		void Normals(
			Mesh *pMesh,
			float *N = NULL);
		void SetKeyPressCallback(
			void (*f)(vtkObject *caller, unsigned long eid, void *clientdata, void *calldata),
			void *clientData);
		void SetMouseRButtonDownCallback(
			void (*f)(vtkObject *caller, unsigned long eid, void *clientdata, void *calldata),
			void *clientData);
		void SetMouseRButtonDownCallback(
			std::function<void(vtkIdType, double *, void *callData)> func,
			void *callData);
		void SetMouseMButtonDownCallback(
			void (*f)(vtkObject *caller, unsigned long eid, void *clientdata, void *calldata),
			void *clientData);
		void SetText(char *textIn);
		void SetPointPicker();
		void SetCellPicker();
		void AssociateActorWithPointPicker(vtkSmartPointer<vtkActor> actor);
		void Run();
		void Clear();
		void Clear(std::vector<vtkSmartPointer<vtkActor>> actors);
		void PaintPoint(
			int iPt,
			vtkSmartPointer<vtkPolyData> &pd,
			unsigned char *Color,
			Figure *pFig = NULL);
		void PaintPointSet(
			QList<QLIST::Index2> *piPtList,
			vtkSmartPointer<vtkPolyData> &pd,
			unsigned char *Color,
			Figure *pFig = NULL,
			int *mapPixToPolyData = NULL);
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
		void AddMesh(
			vtkSmartPointer<vtkPolyData> &pd,
			double *color = NULL);
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
		vtkSmartPointer<vtkActor> DisplayBox(
			double a,
			double b,
			double c,
			Pose3D *pPose,
			double red,
			double green,
			double blue,
			bool bWireFrame = false,
			float lineWidth = 1.0);
		vtkSmartPointer<vtkActor> DisplayCylinder(
			double r,
			double h,
			Pose3D *pPose,
			int resolution,
			double red,
			double green,
			double blue);
		void DisplaySphere(
			float *P,
			float r,
			int resolution);
		void DisplayEllipsoid(
			float *P,
			float *C,
			float r);
		template <typename PointCoordinateType, typename PointType>
		vtkSmartPointer<vtkActor> DisplayPointSet(
			Array<PointType> pointArray,
			unsigned char *color,
			float pointMarkerSize,
			bool bMultiColor = false,
			bool bLabels = false,
			vtkSmartPointer<vtkActor2D> *pLabelActor = NULL)
		{
			vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();

			vtkSmartPointer<vtkPolyData> ptsPolyData = vtkSmartPointer<vtkPolyData>::New();

			vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

			vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
			colors->SetNumberOfComponents(3);
			colors->SetName("Colors");

			int iPt;
			PointCoordinateType *P;

			for (iPt = 0; iPt < pointArray.n; iPt++)
			{
				P = pointArray.Element[iPt].P;

				points->InsertNextPoint(P);

				if (bMultiColor)
					colors->InsertNextTypedTuple(color + 3 * iPt);
				else
					colors->InsertNextTypedTuple(color);
			}

			ptsPolyData->SetPoints(points);

			vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter =
				vtkSmartPointer<vtkVertexGlyphFilter>::New();

			vertexFilter->SetInputData(ptsPolyData);

			vertexFilter->Update();

			polyData->ShallowCopy(vertexFilter->GetOutput());

			polyData->SetPoints(points);

			polyData->GetPointData()->SetScalars(colors);

			// Setup the visualization pipeline
			vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();

			mapper->SetInputData(polyData);

			actor = vtkSmartPointer<vtkActor>::New();
			actor->SetMapper(mapper);
			actor->GetProperty()->SetPointSize(pointMarkerSize);

			renderer->AddActor(actor);

			if (bLabels)
			{
				vtkSmartPointer<vtkLabeledDataMapper> labelMapper = vtkSmartPointer<vtkLabeledDataMapper>::New();
				labelMapper->SetInputData(ptsPolyData);
				if (pLabelActor)
				{
					*pLabelActor = vtkSmartPointer<vtkActor2D>::New();
					(*pLabelActor)->SetMapper(labelMapper);
					renderer->AddActor2D(*pLabelActor);
				}
				else
				{
					vtkSmartPointer<vtkActor2D> labelActor = vtkSmartPointer<vtkActor2D>::New();
					labelActor->SetMapper(labelMapper);
					renderer->AddActor2D(labelActor);
				}
			}

			return actor;
		}
		vtkSmartPointer<vtkActor> DisplayLine(
			Point *vertex,
			uchar *color,
			float lineWidth = 1.0f);
		vtkSmartPointer<vtkActor> DisplayLines(
			Array<Point> vertices,
			Array<Pair<int, int>> lines,
			uchar *color,
			float lineWidth = 1.0f,
			bool bMultiColor = false);
		void DisplayReferenceFrames(
			Array<Pose3D> referenceFrames,
			double axesLength);
		vtkSmartPointer<vtkActor> DisplayReferenceFrame(
			Pose3D *pReferenceFrame,
			double axesLength);
		vtkSmartPointer<vtkActor2D> DisplayLabels(vtkSmartPointer<vtkPolyData> ptsPolyData);
		vtkSmartPointer<vtkActor2D> DisplayLabels(Array<Point> points);
		vtkSmartPointer<vtkActor> DisplayVoxels(
			Array3D<uchar> voxels,
			uchar *faceColor = NULL,
			float opacity = 1.0f,
			uchar *edgeColor = NULL,
			bool bHideEdges = false);
		void DisplayOrganizedOrientedPC(
			Array2D<OrientedPoint> PC,
			IplImage *rgbImage = NULL,
			uchar *pointColorIn = NULL,
			float normalLen = 0.05f,
			uchar *normalColorIn = NULL);
		void DisplaySphereGrid();
		void DisplaySphericalHistogram(Array<Pair<Vector3<float>, float>> vectors);
		void DisplayMesh(Mesh *pMesh);

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
		vtkSmartPointer<vtkCellPicker> cellPicker;
		vtkSmartPointer<vtkCornerAnnotation> text;
		vtkSmartPointer<vtkCallbackCommand> keypressCallback;
		vtkSmartPointer<vtkCallbackCommand> mouseRButtonDownCallback;
		vtkSmartPointer<vtkCallbackCommand> mouseMButtonDownCallback;
		vtkSmartPointer<vtkOrientationMarkerWidget> widget;
		vtkSmartPointer<PointPickCallback> pointPickCallback;
		double normalLength;
		bool bNormals;
		bool bNormalsVisible;
		bool b3D;
		bool b2D;
		std::vector<Figure *> figures;
	};
}
