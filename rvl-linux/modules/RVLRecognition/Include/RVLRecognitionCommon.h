namespace RVL
{
	class Grid
	{
	public:
		Grid();
		virtual ~Grid();
		void Create(
			int w,
			int h,
			int cellSizeIn,
			int maxnPts);
		void Create(
			Array<Point> points,
			Camera *pCameraIn,
			int cellSizeIn);
		void Clear();
		//void GetPointsWithinBlock(
		//	Box<float> box,
		//	Array<int>& points);
		void Add(
			int u,
			int v,
			int iPt);
		bool GetNeighbors(
			float u,
			float v,
			Array<int>& points);
		bool GetNeighbors(
			float* P,
			Array<int>& points);
		void SubSample(
			Array<Point> srcPoints,
			float normalDiffThr,
			Array<int>& pointSubset);

	public:
		int cellSize;
		Camera* pCamera;
		int w;
		int h;
		Array2D<QList<QLIST::Index>> cells;
		Rect<int> idxRect;
		QLIST::Index* mem;
		float maxPtDist;

	private:
		float fCellSize;
		QLIST::Index* pPtIdx;
	};

	class VoxelGrid
	{
	public:
		VoxelGrid();
		virtual ~VoxelGrid();
		void Create(
			Array<Point> points,
			float cellSize,
			int* ptIdx = NULL);
		void Clear();
		void GetPointsWithinBlock(
			Box<float> box,
			Array<int>& points);
		void GetNeighbors(
			float* P,
			Array<int>& points);
		void GetPointsWithinSphere(
			Array<Point> srcPoints,
			float* P,
			float r,
			Array<int>& tgtPoints,
			int* pointBuffMem);
		void SubSample(
			Array<Point> srcPoints,
			float normalDiffThr,
			Array<int>& pointSubset);

	public:
		float cellSize;
		Box<float> BBox;
		Box<int> idxBox;
		float P0[3];
		Array3D<QList<QLIST::Index>> cells;
		QLIST::Index* mem;
		float maxPtDist;
	};

	class PointAssociationData
	{
	public:
		PointAssociationData();
		virtual ~PointAssociationData();
		void Create(
			Mesh* pMMesh,
			Mesh* pQMesh,
			bool bNormals = false);
		void Create(
			int nMPts,
			int nQPts,
			bool bNormals = false);
		void Clear();

	public:
		int* MNN;
		int* QNN;
		float* MNNDist;
		float* QNNDist;
		Array<int> MPts;
		float* PMQ;
		Array<int> explainedQPts;
		Array<int> associatedMPts;
	};

	namespace RECOG
	{
		struct SceneFittingError
		{
			float eP;
			float csN;
			float dz;
		};

		void CreateDilatedDepthImage(
			Mesh *pMesh,
			cv::Mat &depth);
		bool InitZBuffer(
			Mesh* pMesh,
			int sceneSamplingResolution,
			Array2D<Point> &ZBuffer,
			Array<int> &ZBufferActivePtArray,
			int* &subImageMap);
		float EvaluateHypothesis2(
			Mesh* pMesh,
			SurfelGraph *pSurfels,
			bool* surfelMask,
			Array2D<Point> ZBuffer,
			Array<int> ZBufferActivePtArray,
			int* subImageMap,
			int * image3x3Neighborhood,
			float maxe,
			float transparencyDepthThr,
			int& nTransparentPts,
			int* SMCorrespondence,
			RECOG::SceneFittingError* errorRecord);
		float EvaluateHypothesis3(
			Mesh* pMesh,
			SurfelGraph* pSurfels,
			bool* surfelMask,
			Array2D<Point> ZBuffer,
			Array<int> ZBufferActivePtArray,
			int* subImageMap,
			int* image3x3Neighborhood,
			float maxe,
			float transparencyDepthThr,
			int& nTransparentPts,
			int* SMCorrespondence,
			RECOG::SceneFittingError* errorRecord);
		void DisplayHypothesisEvaluation(
			Visualizer* pVisualizer,
			Mesh *pMesh,
			Array2D<Point> ZBuffer,
			Array<int> ZBufferActivePtArray,
			int* subImageMap,
			int* SMCorrespondence,
			int nTransparentPts,
			vtkSmartPointer<vtkActor>* actor);
		void DisplayHypothesisEvaluation2(
			Visualizer* pVisualizer,
			Mesh* pMesh,
			Array2D<Point> ZBuffer,
			Array<int> ZBufferActivePtArray,
			int* subImageMap,
			int* SMCorrespondence,
			int nTransparentPts,
			vtkSmartPointer<vtkActor>* actor);
	}
}
