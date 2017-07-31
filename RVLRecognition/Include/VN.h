#pragma once

namespace RVL
{
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
			};
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
			float voxelSize = 5.0f,
			int sampleVoxelDistance = 2,
			Visualizer *pVisualizer = NULL);
		void DisplaySampledMesh(
			Visualizer *pVisualizer,
			Array3D<RECOG::VN_::Voxel> volume,
			float voxelSize,
			float *P0);
	};
}

