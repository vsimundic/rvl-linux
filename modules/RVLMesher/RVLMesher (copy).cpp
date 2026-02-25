// RVLMesher.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"
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

#define RVLMESHER_FLAG_VISUALIZATION	0x00000001
#define RVLMESHER_METHOD_3DDELAUNAY		0x00000000
#define RVLMESHER_METHOD_VOXELRG		0x00000001
#define RVLMESHER_METHOD_LABELPARTS		0x00000002
#define RVLMESHER_METHOD_POINT_DENSITY	0x00000003
#define RVLMESHER_METHOD_ORIENTPTDIST	0x00000004

using namespace RVL;

void CreateParamList(
	CRVLParameterList *pParamList,
	CRVLMem *pMem,
	char **pSceneSequenceFileName,
	DWORD &method,
	DWORD &flags,
	float &SDFSurfaceValue,
	float &resolution,
	float &ptDensitySigp,
	bool &bRoom)
{
	pParamList->m_pMem = pMem;

	RVLPARAM_DATA *pParamData;

	pParamList->Init();

	pParamData = pParamList->AddParam("SceneSequenceFileName", RVLPARAM_TYPE_STRING, pSceneSequenceFileName);
	pParamData = pParamList->AddParam("Method", RVLPARAM_TYPE_ID, &method);
	pParamList->AddID(pParamData, "3DDELAUNAY", RVLMESHER_METHOD_3DDELAUNAY);
	pParamList->AddID(pParamData, "VOXELRG", RVLMESHER_METHOD_VOXELRG);
	pParamList->AddID(pParamData, "LABELPARTS", RVLMESHER_METHOD_LABELPARTS);
	pParamList->AddID(pParamData, "PTDENSITY", RVLMESHER_METHOD_POINT_DENSITY);
	pParamList->AddID(pParamData, "ORIENTPTDIST", RVLMESHER_METHOD_ORIENTPTDIST);
	pParamData = pParamList->AddParam("Visualization", RVLPARAM_TYPE_FLAG, &flags);
	pParamList->AddID(pParamData, "yes", RVLMESHER_FLAG_VISUALIZATION);
	pParamData = pParamList->AddParam("Mesher.SDFSurfaceValue", RVLPARAM_TYPE_FLOAT, &SDFSurfaceValue);
	pParamData = pParamList->AddParam("Mesher.resolution", RVLPARAM_TYPE_FLOAT, &resolution);
	pParamData = pParamList->AddParam("Mesher.pointDensity.sigp", RVLPARAM_TYPE_FLOAT, &ptDensitySigp);
	pParamData = pParamList->AddParam("Mesher.room", RVLPARAM_TYPE_BOOL, &bRoom);
}

int main(int argc, char ** argv)
{
	CRVLMem mem0;

	mem0.Create(10000000);

	char *sceneSequenceFileName = NULL;
	DWORD method;
	float SDFSurfaceValue = 0.0f;
	float resolution = 0.005f;
	float ptDensitySigp = 0.01f;
	DWORD flags = 0x00000000;
	bool bRoom = false;

	CRVLParameterList ParamList;

	CreateParamList(&ParamList,
		&mem0,
		&sceneSequenceFileName,
		method,
		flags,
		SDFSurfaceValue,
		resolution,
		ptDensitySigp,
		bRoom);

	ParamList.LoadParams("RVLMesher.cfg");

	if (sceneSequenceFileName == NULL)
	{
		printf("No scene specified!\n");

		return 1;
	}

	MESH::Tetrahedrons tetrahedrons;

	Visualizer visualizer;

	visualizer.Create();

	FileSequenceLoader sceneSequence;

	sceneSequence.Init(sceneSequenceFileName);

	char filePath[200];
	Mesh mesh;
	bool bSaveMesh;

	while (sceneSequence.GetNextPath(filePath))
	{
		printf("Scene: %s:\n", filePath);

		if (method == RVLMESHER_METHOD_3DDELAUNAY)
		{
			MESH::LoadTetrahedrons(filePath, tetrahedrons);

			printf("Creating visible surface mesh...");

			mesh.pPolygonData = MESH::CreateVisibleSurfaceMeshFromTetrahedrons(tetrahedrons, resolution, SDFSurfaceValue, bRoom);

			bSaveMesh = true;
		}
		else if (method == RVLMESHER_METHOD_VOXELRG)
		{
			char *fileExtension = RVLGETFILEEXTENSION(filePath);

			if (strcmp(fileExtension, "obj") == 0)
			{
				vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
				reader->SetFileName(filePath);
				reader->Update();
				vtkSmartPointer<vtkPolyData> pPolygonDataSrc = reader->GetOutput();

				//mesh.pPolygonData = MESH::CreateVisibleSurfaceMesh2(pPolygonDataSrc, resolution, SDFSurfaceValue);

				mesh.pPolygonData = pPolygonDataSrc;
			}
			else if (strcmp(fileExtension, "ply") == 0)
			{
				vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
				reader->SetFileName(filePath);
				reader->Update();
				vtkSmartPointer<vtkPolyData> pPolygonDataSrc = reader->GetOutput();

				mesh.pPolygonData = MESH::CreateVisibleSurfaceMesh2(pPolygonDataSrc, resolution, SDFSurfaceValue);
			}

			bSaveMesh = true;
		}
		else if (method == RVLMESHER_METHOD_LABELPARTS)
		{
			//Read dense point cloud
			vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
			reader->SetFileName(filePath);
			reader->Update();
			vtkSmartPointer<vtkPolyData> pDensePolygonData = reader->GetOutput();

			char *PTSFileName = RVLCreateFileName(filePath, ".ply", -1, ".pts");

			vtkSmartPointer<vtkSimplePointsReader> reader2 =	vtkSmartPointer<vtkSimplePointsReader>::New();
			reader2->SetFileName(PTSFileName);
			//reader->SetFileName(filePath);
			reader2->Update();
			vtkSmartPointer<vtkPolyData> pSparsePolygonData = reader2->GetOutput();

			char *labelFileName = RVLCreateFileName(filePath, ".ply", -1, ".seg");

			MESH::LabelParts(pDensePolygonData, pSparsePolygonData, labelFileName);
			MESH::SparsePointsCorrespondences(pDensePolygonData, pSparsePolygonData, labelFileName);

			bSaveMesh = false;
		}
		else if (method == RVLMESHER_METHOD_POINT_DENSITY)
		{
			char *fileExtension = RVLGETFILEEXTENSION(filePath);

			if (strcmp(fileExtension, "ply") == 0)
			{
				vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
				reader->SetFileName(filePath);
				reader->Update();
				vtkSmartPointer<vtkPolyData> pPolygonDataSrc = reader->GetOutput();

				float P0[3];
				Box<float> boundingBox;
				float voxelSize;
				Array3D<float> SDF;

				MESH::PointTDF(pPolygonDataSrc, resolution, 0.5f, ptDensitySigp, SDF, P0, boundingBox, voxelSize);

				mesh.pPolygonData = DisplayIsoSurface(SDF, P0, voxelSize, SDFSurfaceValue);

				delete[] SDF.Element;

				bSaveMesh = true;
			}
		}
		else if (method == RVLMESHER_METHOD_ORIENTPTDIST)
		{
			char *fileExtension = RVLGETFILEEXTENSION(filePath);

			if (strcmp(fileExtension, "ply") == 0)
			{
				vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
				reader->SetFileName(filePath);
				reader->Update();
				vtkSmartPointer<vtkPolyData> pPolygonDataSrc = reader->GetOutput();

				float P0[3];
				Box<float> boundingBox;
				float voxelSize;
				Array3D<float> SDF;
				float *W;

				MESH::OrientedPointDF(pPolygonDataSrc, resolution, 0.5f, ptDensitySigp, SDF, W, P0, boundingBox, voxelSize);
				
				mesh.pPolygonData = DisplayIsoSurface(SDF, P0, voxelSize, SDFSurfaceValue, W);

				delete[] SDF.Element;
				delete[] W;

				bSaveMesh = true;
			}
		}

		printf("completed.\n");

		if (flags & RVLMESHER_FLAG_VISUALIZATION)
		{
			vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
			mapper->SetInputData(mesh.pPolygonData);
			vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
			actor->SetMapper(mapper);

			visualizer.renderer->AddActor(actor);

			visualizer.Run();

			visualizer.renderer->RemoveAllViewProps();
		}

		if (bSaveMesh)
		{
			char *PLYFileName = RVLCreateFileName(filePath, ".ply", -1, "_vs.ply");

			printf("Saving mesh to %s...", PLYFileName);

			mesh.SavePolyDataToPLY(PLYFileName);

			printf("completed.\n");

			delete[] PLYFileName;
		}
	}

	if (sceneSequenceFileName)
		delete[] sceneSequenceFileName;

	return 0;
}

