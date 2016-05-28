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
}