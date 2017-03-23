namespace RVL
{
	int PCLLoadPCD(
		char *FileName,
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PC);

	int PCLMeshToPolygonData(
		pcl::PolygonMesh &mesh,
		vtkSmartPointer< vtkPolyData > &polygonData);

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
}