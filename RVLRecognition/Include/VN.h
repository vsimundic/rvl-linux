#pragma once

namespace RVL
{
	// Move to RVL3DTools.h.

	template <typename T>
	void ExpandBox(Box<T> *pBox, T extension)
	{
		pBox->minx -= extension;
		pBox->maxx += extension;
		pBox->miny -= extension;
		pBox->maxy += extension;
		pBox->minz -= extension;
		pBox->maxz += extension;
	}

	namespace RECOG
	{
		namespace VN_
		{
			struct Voxel
			{
				QList<QLIST::Index> PtList;
				int voxelDistance;
			};

			struct Sample
			{
				float P[3];
				int iFeature;
				float SDF;
			};

			struct Node
			{
				int operation;
				int iFeature;
				float fOperation;
				float output;
				bool bOutput;
				int iActiveFeature;
			};

			typedef QLIST::Entry<Pair<int, int>> Edge;
		}
	}

	class VN
	{
	public:
		VN();
		virtual ~VN();
		void Create(
			Mesh *pMesh,
			SurfelGraph *pSurfels,
			CRVLMem *pMem,
			float voxelSize = 5.0f,
			int sampleVoxelDistance = 2,
			float eps = 2.0f,
			Visualizer *pVisualizer = NULL);
		void Display(
			Visualizer *pVisualizer,
			Box<float> box,
			float resolution);
		void DisplaySampledMesh(
			Visualizer *pVisualizer,
			Array3D<RECOG::VN_::Voxel> volume,
			float voxelSize,
			float *P0);

	public:
		Array<RECOG::VN_::Node> NodeArray;
		QList<QLIST::Entry<Pair<int, int>>> EdgeList;
		int nFeatures;
		Array3D<RECOG::VN_::Voxel> volume;
		float voxelSize;
		float P0[3];
		SurfelGraph *pFeatures;
		int iy;
		Box<float> boundingBox;
	};
}

