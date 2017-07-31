//#include "stdafx.h"
#include "RVLVTK.h"
#include <vtkTriangle.h>
#include <vtkVertexGlyphFilter.h>
#include "RVLCore2.h"
#include "Util.h"
#include "Space3DGrid.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
#include "SurfelGraph.h"
#include "ObjectGraph.h"
#include "PlanarSurfelDetector.h"
#include "RVLRecognition.h"
#include "PSGMCommon.h"
#include "CTISet.h"
#include "VertexGraph.h"
#include "TG.h"
#include "TGSet.h"
#include "PSGM.h"
#include "ObjectDetector.h"
#include <Eigen\Eigenvalues>
#include "VN.h"

using namespace RVL;
using namespace RECOG;

VN::VN()
{
}


VN::~VN()
{
}

void VN::Create(
	Mesh *pMesh,
	SurfelGraph *pSurfels,
	float voxelSize,
	int sampleVoxelDistance,
	Visualizer *pVisualizer)
{
	Box<float> boundingBox;

	pMesh->BoundingBox(&boundingBox);

	int nx = (int)ceil(0.5f * (boundingBox.maxx - boundingBox.minx) / voxelSize) + sampleVoxelDistance;
	int ny = (int)ceil(0.5f * (boundingBox.maxy - boundingBox.miny) / voxelSize) + sampleVoxelDistance;
	int nz = (int)ceil(0.5f * (boundingBox.maxz - boundingBox.minz) / voxelSize) + sampleVoxelDistance;

	float a = voxelSize * (float)nx;
	float b = voxelSize * (float)ny;
	float c = voxelSize * (float)nz;

	float center[3];

	BoxCenter<float>(&boundingBox, center);

	Box<float> box;

	box.minx = center[0] - a;
	box.miny = center[1] - b;
	box.minz = center[2] - c;
	box.maxx = center[0] + a;
	box.maxy = center[1] + b;
	box.maxz = center[2] + c;

	Array3D<RECOG::VN_::Voxel> volume;

	volume.a = 2 * nx;
	volume.b = 2 * ny;
	volume.c = 2 * nz;

	int nVoxels = volume.a * volume.b * volume.c;

	volume.Element = new RECOG::VN_::Voxel[nVoxels];

	//int maxVoxelDistance = volume.a + volume.b + volume.c;

	int maxVoxelDistance = sampleVoxelDistance + 1;

	int i;
	QList<QLIST::Index> *pPtList;
	RECOG::VN_::Voxel *pVoxel;

	for (i = 0; i < nVoxels; i++)
	{
		pVoxel = volume.Element + i;

		pPtList = &(pVoxel->PtList);

		RVLQLIST_INIT(pPtList);

		pVoxel->voxelDistance = -1;
	}

	QLIST::Index *PtMem = new QLIST::Index[pMesh->NodeArray.n];

	QLIST::Index *pPtIdx = PtMem;

	float *P;
	int iPt;
	int j, k;

	for (iPt = 0; iPt < pMesh->NodeArray.n; iPt++)
	{
		P = pMesh->NodeArray.Element[iPt].P;

		i = (int)floor((P[0] - box.minx) / voxelSize);
		j = (int)floor((P[1] - box.miny) / voxelSize);
		k = (int)floor((P[2] - box.minz) / voxelSize);

		pVoxel = RVL3DARRAY_ELEMENT(volume, i, j, k);

		pPtList = &(pVoxel->PtList);

		pPtIdx->Idx = iPt;

		RVLQLIST_ADD_ENTRY(pPtList, pPtIdx);

		pPtIdx++;
	}

	int *RGBuff = new int[nVoxels];

	int *pPut = RGBuff;
	int *pFetch = RGBuff;

	Array<int> zeroDistanceVoxelArray;

	zeroDistanceVoxelArray.Element = new int[nVoxels];

	zeroDistanceVoxelArray.n = 0;

	*(pPut++) = 0;

	int dijk[][3] = {
		{ -1, 0, 0 },
		{ 1, 0, 0 },
		{ 0, -1, 0 },
		{ 0, 1, 0 },
		{ 0, 0, -1 },
		{ 0, 0, 1 } };

	int iVoxel, iVoxel_;
	int i_, j_, k_, l;

	while (pPut > pFetch)
	{
		iVoxel = (*pFetch++);

		RVL3DARRAY_INDICES(volume, iVoxel, i, j, k);

		for (l = 0; l < 6; l++)
		{
			i_ = i + dijk[l][0];
			j_ = j + dijk[l][1];
			k_ = k + dijk[l][2];

			if (i_ >= 0 && i_ < volume.a && j_ >= 0 && j_ < volume.b && k_ >= 0 && k_ < volume.c)
			{
				iVoxel_ = RVL3DARRAY_INDEX(volume, i_, j_, k_);

				pVoxel = volume.Element + iVoxel_;

				if (pVoxel->voxelDistance >= 0)
					continue;

				if (pVoxel->PtList.pFirst)
				{
					pVoxel->voxelDistance = 0;

					zeroDistanceVoxelArray.Element[zeroDistanceVoxelArray.n++] = iVoxel_;
				}
				else
				{
					pVoxel->voxelDistance = maxVoxelDistance;

					*(pPut++) = iVoxel_;
				}
			}
		}
	}

	Array<RECOG::VN_::Sample> sampleArray;

	sampleArray.Element = new RECOG::VN_::Sample[nVoxels];

	sampleArray.n = 0;

	float halfVoxelSize = 0.5f * voxelSize;

	float P0[3];

	P0[0] = box.minx + halfVoxelSize;
	P0[1] = box.miny + halfVoxelSize;
	P0[2] = box.minz + halfVoxelSize;

	pPut = RGBuff + zeroDistanceVoxelArray.n;

	pFetch = RGBuff;

	memcpy(RGBuff, zeroDistanceVoxelArray.Element, zeroDistanceVoxelArray.n * sizeof(int));

	int voxelDistance;

	while (pPut > pFetch)
	{
		iVoxel = (*pFetch++);

		pVoxel = volume.Element + iVoxel;

		voxelDistance = pVoxel->voxelDistance + 1;

		RVL3DARRAY_INDICES(volume, iVoxel, i, j, k);

		for (l = 0; l < 6; l++)
		{
			i_ = i + dijk[l][0];
			j_ = j + dijk[l][1];
			k_ = k + dijk[l][2];

			if (i_ >= 0 && i_ < volume.a && j_ >= 0 && j_ < volume.b && k_ >= 0 && k_ < volume.c)
			{
				iVoxel_ = RVL3DARRAY_INDEX(volume, i_, j_, k_);

				pVoxel = volume.Element + iVoxel_;

				if (pVoxel->voxelDistance > voxelDistance)
				{
					pVoxel->voxelDistance = voxelDistance;

					*(pPut++) = iVoxel_;

					if (voxelDistance == sampleVoxelDistance)
					{
						P = sampleArray.Element[sampleArray.n].P;

						P[0] = (float)i_ * voxelSize;
						P[1] = (float)j_ * voxelSize;
						P[2] = (float)k_ * voxelSize;

						RVLSUM3VECTORS(P, P0, P);

						sampleArray.n++;
					}
				}
			}
		}
	}

	if (pVisualizer)
	{
		DisplaySampledMesh(pVisualizer, volume, voxelSize, P0);

		unsigned char color[] = { 0, 128, 255 };

		pVisualizer->DisplayPointSet<float, VN_::Sample>(sampleArray, color, 6.0f);
	}

	delete[] volume.Element;
	delete[] PtMem;
	delete[] RGBuff;
	delete[] zeroDistanceVoxelArray.Element;
	delete[] sampleArray.Element;
}

void VN::DisplaySampledMesh(
	Visualizer *pVisualizer,
	Array3D<RECOG::VN_::Voxel> volume,
	float voxelSize,
	float *P0)
{
	Array3D<float> f;

	f.a = volume.a;
	f.b = volume.b;
	f.c = volume.c;

	int nVoxels = volume.a * volume.b * volume.c;

	f.Element = new float[nVoxels];

	int iVoxel;

	for (iVoxel = 0; iVoxel < nVoxels; iVoxel++)
		f.Element[iVoxel] = (volume.Element[iVoxel].voxelDistance > 0 ? 1.0f : -1.0f);

	vtkSmartPointer<vtkPolyData> polyData = DisplayIsoSurface(f, P0, voxelSize, 0.0f);

	// Create a mapper and actor.
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(polyData);
	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);

	pVisualizer->renderer->AddActor(actor);

	delete[] f.Element;
}

