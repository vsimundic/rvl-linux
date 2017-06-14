#pragma once

#define RVLPCLMESHBUILDER_FLAG_BILATERAL_FILTER		0x00000001
#define RVLPCLMESHBUILDER_FLAG_ORGANIZED_PC			0x00000002

namespace RVL
{
	bool LoadMesh(
		void *vpMeshBuilder,
		char *FileName,
		Mesh *pMesh,
		bool bSavePLY);

	class PCLMeshBuilder
	{
	public:
		PCLMeshBuilder();
		virtual ~PCLMeshBuilder();
		void CreateMesh(
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC,
			pcl::PolygonMesh &mesh);
		bool CreateMesh(
			vtkSmartPointer<vtkPolyData> pPolygonData,
			pcl::PolygonMesh &mesh);
		void CreateParamList(CRVLMem *pMem);
		bool Load(
			char *FileName,
			Mesh *pMesh,
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC,
			pcl::PolygonMesh &PCLMesh,
			bool bSavePLY);
		bool Load(
			char *FileName,
			Mesh *pMesh,
			bool bSavePLY);

	public:
		DWORD flags;
		double sigmaS;
		double sigmaR;
		double normalEstR;		
		int width;
		int height;
		CRVLParameterList ParamList;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC;
		pcl::PolygonMesh PCLMesh;
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

