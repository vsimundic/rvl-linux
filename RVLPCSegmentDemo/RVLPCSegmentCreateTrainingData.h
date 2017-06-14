int DetPrimaryGTObj(Surfel *pSurfel, cv::Mat labGTImg, int noObj);
//void SetPrimaryGTObj(Surfel *pSurfel, cv::Mat labGTImg, int noObj);
float GetShortestDistanceToBoundary(int x, int y, Array<MeshEdgePtr *> &BoundaryArray);
void DrawSurfelBoundary(Array<MeshEdgePtr *> &BoundaryArray, cv::Mat img);
bool GetSurfelGTValidity(Surfel *pSurfel, cv::Mat labGTImg, float thr);
void FindSurfelNeighbours(std::vector<Surfel*> &nList, Surfel *pSurfel, SurfelGraph *surfels, int thr = 5);
void SetSurfelImgAdjacency(Surfel *pSurfel, SurfelGraph *surfels, Mesh *mesh, int thr = 5);
void GetSurfelImgCentroid(int &x, int &y, Surfel *pSurfel);
void AddVTKLine(float *P1, float *P2, vtkSmartPointer<vtkRenderer> renderer, double *color);
cv::Mat GenColoredGTImg(cv::Mat labImg);
cv::Mat GenColoredSegmentationImg(SurfelGraph *surfels);
void DrawSurfelImgAdjacencyOpenCV(cv::Mat img, SurfelGraph *surfels);
void RenderSurfelImgAdjacencyVTK(vtkSmartPointer<vtkRenderer> renderer, SurfelGraph *surfels);
void PreprocessGTLab(cv::Mat GTLabImg, cv::Mat GTDisparityImg);
//void DetermineImgAdjDescriptors(Surfel *pSurfel, Mesh *mesh);
void ComputeRelationFeatures(
	SurfelGraph *pSurfels,
	Mesh *pMesh);
//void GenerateSSF(SurfelGraph *surfels, std::string filename);
//void GenerateSSF(SurfelGraph *surfels, std::string filename, int minSurfelSize, bool checkbackground = true);
void RunSeg2Bench(bool save = false);
//cv::Mat GenColoredSurfelImgFromSSF(std::shared_ptr<SceneSegFile::SceneSegFile> ssf);
cv::Mat GenColoredSegmentationImgFromObjectGraph_SSF(SURFEL::ObjectGraph* objects);
//cv::Mat GenColoredSegmentationImgFromObjectGraph(SURFEL::ObjectGraph* objects);
