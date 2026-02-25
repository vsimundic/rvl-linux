#pragma once

namespace RVL
{
	template<typename PointType, typename PointCloudPtrType, typename SrcPointType>
	class PCLNeighborhoodTool : public NeighborhoodTool
	{
	public:
		PCLNeighborhoodTool()
		{
		}
		virtual ~PCLNeighborhoodTool()
		{
		}
		void Create(void *vpPointArray)
		{
			PC->clear();

			Array<SrcPointType *> *pPointArray = (Array<SrcPointType *> *)vpPointArray;

			PCLCopyPointsAndNormals<SrcPointType, PointCloudPtrType>(pPointArray, PC);
			
			kdtree.setInputCloud(PC);
		}
		void SetPointCloud(PointCloudPtrType PCIn)
		{
			PC = PCIn;
		}
		void RadiusSearch(
			float *P,
			float radius,
			std::vector<int> &pointIdxRadiusSearch,
			std::vector<float> &pointRadiusSquaredDistance)
		{
			PointType searchPoint;
			searchPoint.x = P[0];
			searchPoint.y = P[1];
			searchPoint.z = P[2];

			kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
		}

	private:
		PointCloudPtrType PC;
		pcl::search::KdTree<PointType> kdtree;
	};
}

