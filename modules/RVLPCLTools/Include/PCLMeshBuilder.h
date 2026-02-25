#pragma once

#define RVLPCLMESHBUILDER_FLAG_BILATERAL_FILTER		0x00000001
#define RVLPCLMESHBUILDER_FLAG_ORGANIZED_PC			0x00000002
#define RVLPCLMESHBUILDER_FLAG_VISIBLE_SURFACE		0x00000004
#define RVLPCLMESHBUILDER_FLAG_CREATE_ORGANIZED_PC	0x00000008
#define RVLPCLMESHBUILDER_FLAG_UVD_SPACE			0x00000010

namespace RVL
{
	bool LoadMesh(
		void *vpMeshBuilder,
		char *FileName,
		Mesh *pMesh,
		bool bSavePLY,
		char *PLYFileName = NULL,
		char *depthFileName = NULL);
	void CreateMesh(
		void *vpMeshBuilder,
		Array2D<short int> *pDepthImage,
		IplImage *pRGBImage,
		Mesh *pMesh,
		bool bSavePLY = false,		
		char *FileName = NULL);

	class PCLMeshBuilder
	{
	public:
		PCLMeshBuilder();
		virtual ~PCLMeshBuilder();
		void Create(
			char *cfgFileName,
			CRVLMem *pMem);
		void CreateMesh(
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC,
			pcl::PolygonMesh &mesh,
			bool bUVDSpace = false);
		bool CreateMesh(
			vtkSmartPointer<vtkPolyData> pPolygonData,
			pcl::PolygonMesh &mesh);
		void CreateParamList(CRVLMem *pMem);
		bool Load(
			char *RGBFileName,
			char *depthFileName,
			Mesh *pMesh,
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC,
			pcl::PolygonMesh &PCLMesh,
			bool bSavePLY,
			char *PLYFileName = NULL);
		bool Load(
			char *FileName,
			Mesh *pMesh,
			bool bSavePLY);
		void CreateOrganizedPC(
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PCSrc,
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PCTgt);
		void CreateMesh(
			Array2D<short int> depthImage_array,
			IplImage *pRGBImage,
			Mesh *pMesh,
			bool bConvertToMetres = true,
			bool bUVDSpace = false);
		void CreateMesh(pcl::PointCloud<pcl::PointXYZRGBA> &PC_);
		void RepareOrganizedPC(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC);
		bool CreatePixelMapFromPolyData(
			vtkSmartPointer<vtkPolyData> pPolygonData,
			int *&pixMap,
			int *&mapPixelsToPolyData);
		void convertMeshfromUVDtoXYZspace(Mesh *pMesh);
		void convertPolygonDatafromUVDtoXYZspace(Mesh *pMesh);
		void convertPCLnormalFromUVDtoXYZspace(float* normalUVDtoXYZ, float* pointUVDtoXYZ);
		void convertPCLpointFromUVDtoXYZspace(float* pointUVDtoXYZ);
				

	public:
		DWORD flags;
		double sigmaS;
		double sigmaR;
		double normalEstR;		
		float organizedFastMeshMaxEdgeLen;
		int width;
		int height;
		CRVLParameterList ParamList;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC;
		pcl::PolygonMesh PCLMesh;
		float voxelSize;
		int nSDFFilter;
		float SDFIsoValue;
		RGBDCamera camera;
		int smoothingFilterNumIterations;
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

