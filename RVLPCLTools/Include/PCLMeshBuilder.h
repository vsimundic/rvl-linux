#pragma once

#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/features/normal_3d_omp.h>

namespace RVL
{
	class PCLMeshBuilder
	{
	public:
		PCLMeshBuilder();
		virtual ~PCLMeshBuilder();
		void CreateMesh(
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC,
			pcl::PolygonMesh &mesh);

	public:
		double sigmaS;
		double sigmaR;
		bool bBilateralFilter;
	};
}

