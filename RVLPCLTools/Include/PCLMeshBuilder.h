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
	};
}

