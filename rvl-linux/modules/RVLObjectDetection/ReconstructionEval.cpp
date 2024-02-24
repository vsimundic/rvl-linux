#include <vtkAutoInit.h>
//VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
#include <vtkOBJReader.h>
#include "RVLCore2.h"
#include "RVLVTK.h"
#include "Util.h"
#include "Space3DGrid.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "ReconstructionEval.h"

using namespace RVL;

ReconstructionEval::ReconstructionEval()
{
	bGT = false;
	grid.Element = NULL;
	resolution = 0.005f;
}


ReconstructionEval::~ReconstructionEval()
{
	RVL_DELETE_ARRAY(grid.Element);
}


void ReconstructionEval::LoadGT(char *GTFileName)
{
	if (GTFileName == NULL) 
		return; 

	// Load models.

	char *modelsFileName = RVLCreateFileName(GTFileName, ".scene.dat", -1, ".model.dat");

	FILE *fpModels = fopen(modelsFileName, "r");

	delete[] modelsFileName;

	models.clear();

	char line[200];

	while (!feof(fpModels))
	{
		fgets(line, 200, fpModels);

		if (strcmp(line, "object ID\ttype\tpar1\tpar2\tpar3\n") == 0)
			break;
	}

	RECONSTRUCT::Model model;

	int iModel;
	char modelName[100];

	while (!feof(fpModels))
	{
		fscanf(fpModels, "%d %s\t%d\t%f\t%f\t%f\n", &iModel, modelName, &(model.type), &(model.param1), &(model.param2), &(model.param3));

		model.param1 *= 0.001f;
		model.param2 *= 0.001f;
		model.param3 *= 0.001f;

		models[iModel] = model;
	}

	fclose(fpModels);

	// Initialize bounding box.

	float P[3];
	RVLNULL3VECTOR(P);

	Box<float> fSceneBBox;
	InitBoundingBox<float>(&fSceneBBox, P);

	// Load scenes.

	FILE *fpScenes = fopen(GTFileName, "r");

	scenes.clear();

	fgets(line, 200, fpScenes);

	float P1[3];
	P1[2] = 0.0f;
	float P2[3];
	P2[2] = 0.0f;

	float RTO[9];
	float *XOT = RTO;
	float *YOT = RTO + 3;
	float *ZOT = RTO + 6;

	float U[3];
	RVLSET3VECTOR(U, 0.0f, 0.0f, 1.0f);

	float PBox[3 * 8];
	int i, j, k, iScene, type;
	RECONSTRUCT::GTData GT;
	float fTmp, a, b, c, L, a_, b_, c_, r, h, h_;
	float tOT[3], tOT1[3], tOT2[3], V3Tmp[3], V3Tmp2[3], V3Tmp3[3];
	Box<float> modelBBox;
	float *P_;

	while (!feof(fpScenes))
	{
		fscanf(fpScenes, "%d\t%d\t%d\t%f\t%f\t%f\t%f\n", &iScene, &(GT.iModel), &type, P1, P1 + 1, P2, P2 + 1);

		RVLSCALE3VECTOR(P1, 0.001f, P1);
		RVLSCALE3VECTOR(P2, 0.001f, P2);

		if (models[GT.iModel].type == RVLRECONSTRUCTION_MODEL_TYPE_BOX)
		{
			a = models[GT.iModel].param1;
			b = models[GT.iModel].param2;
			c = models[GT.iModel].param3;

			a_ = 0.5f * a;
			b_ = 0.5f * b;
			c_ = 0.5f * c;

			RVLDIF3VECTORS(P2, P1, V3Tmp);
			RVLNORM3(V3Tmp, L);

			if (fabs(L - a) < fabs(L - b))
			{
				RVLCOPY3VECTOR(V3Tmp, XOT);
				RVLCOPY3VECTOR(U, ZOT);
				RVLCROSSPRODUCT3(ZOT, XOT, YOT);
				RVLSCALE3VECTOR(XOT, a_, V3Tmp);
				RVLSCALE3VECTOR(YOT, b_, V3Tmp2);
				RVLSCALE3VECTOR(ZOT, c_, V3Tmp3);
			}
			else
			{
				RVLCOPY3VECTOR(V3Tmp, YOT);
				RVLCOPY3VECTOR(U, XOT);
				RVLCROSSPRODUCT3(XOT, YOT, ZOT);
				RVLSCALE3VECTOR(XOT, a_, V3Tmp3);
				RVLSCALE3VECTOR(YOT, b_, V3Tmp);
				RVLSCALE3VECTOR(ZOT, c_, V3Tmp2);
			}
			RVLSUM3VECTORS(P1, V3Tmp, tOT1);
			RVLSUM3VECTORS(tOT1, V3Tmp2, tOT1);
			RVLDIF3VECTORS(P2, V3Tmp, tOT2);
			RVLSUM3VECTORS(tOT2, V3Tmp2, tOT2);
			RVLSUM3VECTORS(tOT1, tOT2, tOT);
			RVLSCALE3VECTOR(tOT, 0.5f, tOT);
			RVLSUM3VECTORS(tOT, V3Tmp3, tOT);
			RVLCOPYMX3X3T(RTO, GT.pose.R);
			RVLCOPY3VECTOR(tOT, GT.pose.t);

			modelBBox.maxx = a_;
			modelBBox.maxy = b_;
			modelBBox.maxz = c_;
		}
		else if (models[GT.iModel].type == RVLRECONSTRUCTION_MODEL_TYPE_CYLINDER)
		{
			h = models[GT.iModel].param1;
			r = models[GT.iModel].param2;

			h_ = 0.5f * h;

			RVLCOPY3VECTOR(U, ZOT);
			RVLORTHOGONAL3(ZOT, XOT, i, j, k, fTmp);
			RVLCROSSPRODUCT3(ZOT, XOT, YOT);
			RVLCOPYMX3X3T(RTO, GT.pose.R);
			RVLSCALE3VECTOR(ZOT, h_, V3Tmp3);
			RVLSUM3VECTORS(P1, V3Tmp3, GT.pose.t);

			modelBBox.maxx = r;
			modelBBox.maxy = r;
			modelBBox.maxz = h_;
		}

		modelBBox.minx = -modelBBox.maxx;
		modelBBox.miny = -modelBBox.maxy;
		modelBBox.minz = -modelBBox.maxz;

		BoxVertices<float>(&modelBBox, PBox);

		for (i = 0; i < 8; i++)
		{
			P_ = PBox + 3 * i;
			RVLTRANSF3(P_, GT.pose.R, GT.pose.t, P);
			UpdateBoundingBox<float>(&fSceneBBox, P);
		}

		scenes[iScene].push_back(GT);
	}

	fclose(fpScenes);

	// Create grid.

	sceneBBox.minx = round(fSceneBBox.minx / resolution);
	sceneBBox.maxx = round(fSceneBBox.maxx / resolution);
	sceneBBox.miny = round(fSceneBBox.miny / resolution);
	sceneBBox.maxy = round(fSceneBBox.maxy / resolution);
	sceneBBox.minz = round(fSceneBBox.minz / resolution);
	sceneBBox.maxz = round(fSceneBBox.maxz / resolution);

	grid.a = sceneBBox.maxx - sceneBBox.minx + 1;
	grid.b = sceneBBox.maxy - sceneBBox.miny + 1;
	grid.c = sceneBBox.maxz - sceneBBox.minz + 1;

	RVL_DELETE_ARRAY(grid.Element);

	grid.Element = new uchar[grid.a * grid.b * grid.c];

	// Load camera pose.

	char *cameraFileName = RVLCreateFileName(GTFileName, ".scene.dat", -1, ".cam.dat");

	FILE *fpCamera = fopen(cameraFileName, "r");

	delete[] cameraFileName;

	for (i = 0; i < 3; i++)
		fscanf(fpCamera, "%f\t%f\t%f\t%f\n", cameraPose.R + 3 * i, cameraPose.R + 3 * i + 1, cameraPose.R + 3 * i + 2, cameraPose.t + i);

	fclose(fpCamera);

	bGT = true;
}

void ReconstructionEval::DisplayGT(
	Visualizer *pVisualizer,
	int iScene)
{
	int i, iModel;
	Pose3D *pTOT;
	Pose3D TOC;
	float V3Tmp[3];

	for (i = 0; i < scenes[iScene].size(); i++)
	{
		iModel = scenes[iScene][i].iModel;

		pTOT = &(scenes[iScene][i].pose);

		RVLCOMPTRANSF3DWITHINV(cameraPose.R, cameraPose.t, pTOT->R, pTOT->t, TOC.R, TOC.t, V3Tmp);

		if (models[iModel].type == RVLRECONSTRUCTION_MODEL_TYPE_BOX)
			pVisualizer->DisplayBox(models[iModel].param1, models[iModel].param2, models[iModel].param3, &TOC, 0.0, 0.0, 1.0);
		else if (models[iModel].type == RVLRECONSTRUCTION_MODEL_TYPE_CYLINDER)
			pVisualizer->DisplayCylinder(models[iModel].param2, models[iModel].param1, &TOC, 20, 0.0, 0.0, 1.0);
	}
}

void ReconstructionEval::DisplayGrid(
	Visualizer *pVisualizer,
	Array3D<uchar> grid_,
	uchar *color)
{
	Array<Point> ptArray;

	ptArray.Element = new Point[grid_.a * grid_.b * grid_.c];

	ptArray.n = 0;

	int i, j, k;
	float PT[3], V3Tmp[3];
	float *PC;

	for (k = 0; k < grid_.c; k++)
		for (j = 0; j < grid_.b; j++)
			for (i = 0; i < grid_.a; i++)
				if (*RVL3DARRAY_ELEMENT(grid_, i, j, k))
				{
					PT[0] = resolution * (float(i));
					PT[1] = resolution * (float(j));
					PT[2] = resolution * (float(k));

					PC = ptArray.Element[ptArray.n++].P;

					RVLINVTRANSF3(PT, cameraPose.R, cameraPose.t, PC, V3Tmp);
				}

	pVisualizer->DisplayPointSet<float, Point>(ptArray, color, 2);

	delete[] ptArray.Element;
}

void ReconstructionEval::DisplayGridComparison(
	Visualizer *pVisualizer,
	Array3D<uchar> grid,
	Array3D<uchar> gridGT)
{
	Array<Point> ptArrayTP;

	ptArrayTP.Element = new Point[gridGT.a * gridGT.b * gridGT.c];

	ptArrayTP.n = 0;

	Array<Point> ptArrayFP;

	ptArrayFP.Element = new Point[gridGT.a * gridGT.b * gridGT.c];

	ptArrayFP.n = 0;

	Array<Point> ptArrayFN;

	ptArrayFN.Element = new Point[gridGT.a * gridGT.b * gridGT.c];

	ptArrayFN.n = 0;

	int iCell = 0;

	int i, j, k;
	float PT[3], V3Tmp[3];
	float *PC;

	for (k = 0; k < grid.c; k++)
		for (j = 0; j < grid.b; j++)
			for (i = 0; i < grid.a; i++, iCell++)
			{
				if (grid.Element[iCell] | gridGT.Element[iCell])
				{
					PT[0] = resolution * (float(i));
					PT[1] = resolution * (float(j));
					PT[2] = resolution * (float(k));

					if (gridGT.Element[iCell] == 1)
					{
						if (grid.Element[iCell] == 1)
							PC = ptArrayTP.Element[ptArrayTP.n++].P;
						else
							PC = ptArrayFN.Element[ptArrayFN.n++].P;
					}
					else 
						PC = ptArrayFP.Element[ptArrayFP.n++].P;

					RVLINVTRANSF3(PT, cameraPose.R, cameraPose.t, PC, V3Tmp);
				}
			}

	uchar red[3] = {255, 0, 0};
	uchar green[3] = { 0, 255, 0 };
	uchar blue[3] = {0, 0, 255};

	pVisualizer->DisplayPointSet<float, Point>(ptArrayTP, green, 2);
	pVisualizer->DisplayPointSet<float, Point>(ptArrayFP, red, 2);
	pVisualizer->DisplayPointSet<float, Point>(ptArrayFN, blue, 2);

	delete[] ptArrayTP.Element;
	delete[] ptArrayFP.Element;
	delete[] ptArrayFN.Element;
}

void ReconstructionEval::CreateGTGrid(int iScene)
{
	memset(grid.Element, 0, grid.a * grid.b * grid.c * sizeof(uchar));

	int i, j, k, iObject, iModel;
	Pose3D *pTOT;
	float *ROT, *tOT;
	float PT[3], PO[3], V3Tmp[3];
	float a, b, c, r, h;
	RECONSTRUCT::Model model;

	for (iObject = 0; iObject < scenes[iScene].size(); iObject++)
	{
		iModel = scenes[iScene][iObject].iModel;

		model = models[iModel];

		pTOT = &(scenes[iScene][iObject].pose);

		ROT = pTOT->R;
		tOT = pTOT->t;

		if (model.type == RVLRECONSTRUCTION_MODEL_TYPE_BOX)
		{
			a = 0.5f * model.param1;
			b = 0.5f * model.param2;
			c = 0.5f * model.param3;
		}
		else if (model.type == RVLRECONSTRUCTION_MODEL_TYPE_CYLINDER)
		{
			h = 0.5f * model.param1;
			r = model.param2;
		}

		for (k = 0; k < grid.c; k++)
			for (j = 0; j < grid.b; j++)
				for (i = 0; i < grid.a; i++)
				{
					PT[0] = resolution * (float(i));
					PT[1] = resolution * (float(j));
					PT[2] = resolution * (float(k));
					
					RVLINVTRANSF3(PT, ROT, tOT, PO, V3Tmp);

					if (model.type == RVLRECONSTRUCTION_MODEL_TYPE_BOX)
					{
						if (PO[0] >= -a && PO[0] <= a && PO[1] >= -b && PO[1] <= b && PO[2] >= -c && PO[2] <= c)
							*RVL3DARRAY_ELEMENT(grid, i, j, k) = 1;
					}
					else if (model.type == RVLRECONSTRUCTION_MODEL_TYPE_CYLINDER)
					{
						if (PO[0] * PO[0] + PO[1] * PO[1] <= r * r && PO[2] >= -h && PO[2] <= h)
							*RVL3DARRAY_ELEMENT(grid, i, j, k) = 1;
					}
				}
	}
}

