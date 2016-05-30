#pragma once

#define RVLPCLMESHBUILDER_FLAG_BILATERAL_FILTER		0x00000001

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
		void CreateParamList(CRVLMem *pMem);

	public:
		DWORD flags;
		double sigmaS;
		double sigmaR;
		double normalEstR;		
		CRVLParameterList ParamList;
	private:
		pcl::PointCloud<pcl::PointXYZRGBA> FPC;
		pcl::PointCloud<pcl::Normal> N;
		//pcl::FastBilateralFilter<pcl::PointXYZRGBA> bilateralFilter;
		//pcl::NormalEstimationOMP<pcl::PointXYZRGBA, pcl::Normal> norm_est;
		//pcl::OrganizedFastMesh<pcl::PointXYZRGBA> OFM;
		void *vpBilateralFilter;
		void *vpNormalEstimator;
		void *vpOFM;
		pcl::PCLPointCloud2 N2;
		pcl::PCLPointCloud2 aux;
	};
}

