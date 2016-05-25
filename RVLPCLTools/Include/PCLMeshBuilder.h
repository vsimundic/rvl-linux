#pragma once

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

