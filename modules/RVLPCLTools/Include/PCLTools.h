namespace RVL
{
	int PCLLoadPCD(
		char *FileName,
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC);

	int PCLMeshToPolygonData(
		pcl::PolygonMesh &mesh,
		vtkSmartPointer< vtkPolyData > &polygonData);

	void PolygonDataToPCLPointCloudWithNormals(
		vtkSmartPointer< vtkPolyData > pPolygonData,
		pcl::PointCloud<pcl::PointXYZRGBA>& PC_,
		pcl::PointCloud<pcl::Normal>& N);

	void PCLSavePLY(
		char *FileName,
		pcl::PolygonMesh &mesh);

	//Petra:
	class PCLICPVariants
	{
	public: 
		enum{Point_to_point = 1,
			 Point_to_plane, 
			 Point_to_point_nonlinear,
			 GeneralizedICP //recommended
			};
	};

	//Returns Transformation matrix between source and destination point clouds:
	void PCLICP(
		vtkSmartPointer<vtkPolyData> pdSource,
		vtkSmartPointer<vtkPolyData> pdDestination,
		float *T,
		int maxIterations,
		float maxCorrespondenceDist,
		int ICPvariant,
		double *fitnessScore,
		void *kdTreePtr=NULL);

	template <typename SrcPointType, typename TgtPointCloudPtrType>
	void PCLCopyPointsAndNormals(
		Array<SrcPointType *> *pPoints,
		TgtPointCloudPtrType cloud_destination)
	{
		cloud_destination->width = pPoints->n;
		cloud_destination->height = 1;
		cloud_destination->is_dense = false;
		cloud_destination->points.resize(cloud_destination->width * cloud_destination->height);

		for (int i = 0; i <pPoints->n; i++)
		{
			cloud_destination->points[i].x = pPoints->Element[i]->P[0];
			cloud_destination->points[i].y = pPoints->Element[i]->P[1];
			cloud_destination->points[i].z = pPoints->Element[i]->P[2];

			cloud_destination->points[i].normal_x =pPoints->Element[i]->N[0];
			cloud_destination->points[i].normal_y =pPoints->Element[i]->N[1];
			cloud_destination->points[i].normal_z =pPoints->Element[i]->N[2];
		}
	}
}