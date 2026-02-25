// RVLConvexHull.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
#include "RVLVTK.h"
#include "RVLCore2.h"
#include "Util.h"
#include "Space3DGrid.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"

using namespace RVL;

int main(int argc, char ** argv)
{
	// Remove after finishing Mesh::ConvexHull().

	float P[] = {
		1, 1, 1,
		-1, 1, 1,
		-1, -1, 1,
		1, -1, 1,
		1, 1, -1,
		-1, 1, -1,
		-1, -1, -1,
		1, -1, -1
	};

	Array2D<float> points;

	points.w = 3;
	//points.h = 8;
	//points.Element = P;
	points.h = 30;
	points.Element = new float[3 * points.h];

	CRVLMem memCH;

	memCH.Create(10000000);

	Visualizer visualizerCH;

	visualizerCH.Create();

	visualizerCH.SetMouseRButtonDownCallback(MESH::MouseRButtonDown, &visualizerCH);

	Mesh mesh;
	float *P_;

	for (int k = 0; k < 100; k++)
	{
		for (int i = 0; i < points.h; i++)
		{
			P_ = points.Element + 3 * i;

			P_[0] = 2.0f * ((float)rand() / (float)RAND_MAX) - 1.0f;
			P_[1] = 2.0f * ((float)rand() / (float)RAND_MAX) - 1.0f;
			P_[2] = 2.0f * ((float)rand() / (float)RAND_MAX) - 1.0f;
		}

		//if (k == 4)
		//{
		//	FILE *fpCH = fopen("C:\\RVL\\Debug\\P.txt", "w");

		//	PrintMatrix<float>(fpCH, points.Element, points.h, points.w);

		//	fclose(fpCH);
		//}

		//FILE *fpCH = fopen("C:\\RVL\\Debug\\P.txt", "r");

		//for (int i = 0; i < points.h; i++)
		//{
		//	P_ = points.Element + 3 * i;

		//	fscanf(fpCH, "%f\t%f\t%f\n", P_, P_ + 1, P_ + 2);
		//}

		//fclose(fpCH);

		//for (int k = 4; k <= 8; k++)
		{
			//points.h = 8;

			//mesh.ConvexHull(points, &memCH);
			mesh.ConvexHull(points, &memCH);

			//points.h = 16;

			mesh.CreateVTKPolyData();

			visualizerCH.AddMesh(mesh.pPolygonData);

			//vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
			//mapper->SetInputData(mesh.pPolygonData);
			//vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
			//actor->SetMapper(mapper);

			//visualizerCH.renderer->AddActor(actor);

			uchar color[] = { 0, 128, 255 };

			visualizerCH.DisplayPointSet<float, Point>(mesh.NodeArray, color, 6);

			visualizerCH.Run();

			visualizerCH.renderer->RemoveAllViewProps();
		}
	}

	delete[] points.Element;

	return 0;
}

