#pragma once

#define RVLLOCALIZATION_MODE_TRAINING	0
#define RVLLOCALIZATION_MODE_LOCALIZATION	1

namespace RVL
{
	struct CrspSurfels
	{
		int surfelID_vw1;
		int viewID_1;
		int surfelID_vw2;
		int viewID_2;
	};

	struct crspPts
	{
		int iView;
		int ptID_iView;
		int jView;
		int ptID_jView;
	};

	struct CrspSurfelPairs
	{
		int viewID_1;
		int S1_ID_vw1;
		int S2_ID_vw1;
		int viewID_2;
		int S1_ID_vw2;
		int S2_ID_vw2;
		float sigma_sum;
	};

	struct CrspPose
	{
		int viewID_1;
		int viewID_2;
		float R[9];
		float t[3];
	};

	struct overlappingPoints
	{
		int iView;
		int ptID_iView;
		int jView;
		int ptID_jView;
		Point point_i2j;
		float infoContentFactor;
		float c;
	};

	struct overlappingViews
	{
		int iView;
		int jView;
		float covariance_ij[9];
	};

	class VNLocalizer
	{
	public:			
		VNLocalizer();
		virtual ~VNLocalizer();
		void CreateParamList();
		void Create(char *cfgFileName);
		void Clear();
		//void UpdateMap(Mesh *pMesh, CameraView *pCameraView, char *fileName);	//SUNCG -v1
#ifndef AR20			
		void UpdateMap(Mesh *pMesh, CameraView cameraView, char *fileName);
#endif

		void GetCenteralPointsOfImageCells(Array<Point> &centralPointsArray);

		//new Mateja	
		int ReadCameraFile();
#ifndef AR20			
		void Associate(View*, View*, int, int);
		//bool Match(Surfel*, Surfel*, CameraView*, CameraView*);	// SUNCG -v1
		bool Match(Surfel*, Surfel*, CameraView, CameraView);
		void CorrespondingSurfelPairs(View*, View*, int, int);
		void CreatePairs(View*, std::vector<struct SurfelPair*>&);
		float PairRegValue(Surfel*, Surfel*);
		float SigmaCalc(Surfel *pS1, Surfel *pS2);

		bool PlanarSurfacePairs(Surfel*, Surfel*, Surfel*, Surfel*);

		void TwoPointCloudReg(View*, View*, CrspSurfelPairs*, float *, float *);
		void OptimalRotTrans(View*, Surfel*, View*, Surfel*, float *, float *, float *, float *, float&);
		//float* AngleAxisToRot(float[3], float);
#endif

		//Multi-view

		void GetCenteralPointsFromView(View *view);
		void ProjectCentralPoint(Array<Point> centralPointsArray);
		void UnexploredRegions(ProjectionCube *cube, float *R_cameraView, float *t_cameraView);
		//void CalculateScore(Array<Pair<int, int>> viewDirectionCandidates, ProjectionCube *cube, int cubeSize, float *bestPanTilt);
		bool CalculateScore(Array<Pair<int, int>> viewDirectionCandidates, ProjectionCube *cube, int cubeSize, float overlapInfoThrIn, float wUnexploredIn, float wOverlapIn, float *bestPanTilt, char *resultsFolder);
		void GetPanTiltCandidate(Pair<int, int> viewDirectionCandidate, int cubeSize, float RCube[9], float *PanTilt);
		void InformationContent(View *view);

		void ImagesOverlap();
		void ImagesOverlap_old(View *view_k, int k, View *view_l, int l);
		void ImagesSampling();
		void OverlappingImagesCovariance();
		void InforamtionContentFactor();
		void SamplePoints();
		void DataAssociation();
		void VisualizeCorrespondences(Visualizer *pVisualizer, View *kView, View *lView, std::vector<Pair<int, int>> correspondences);

		// Multi-view ICP
		float MultiPointCloudReg(float* X);
		double MultiPointCloudReg(double* X);
		float MultiViewICP(View* iView, Point *p_i, View* jView, Point *p_j, float d_ik, float* g_ij_, float* h_ij_, float* A_ij_, float* B_ij_, float* D_ij_);
		double MultiViewICP(View* iView, Point *p_i, View* jView, Point *p_j, double d_ik, double* g_ij_, double* h_ij_, double* A_ij_, double* B_ij_, double* D_ij_);
		double MultiViewICP2(View* iView, Point *p_i, View* jView, Point *p_j, double d_ik, double* g_ij_, double* h_ij_, double* A_ij_, double* B_ij_, double* D_ij_);
		void MultiPointCloudCorrection(float *X);
		void MultiPointCloudCorrection(double *X);
		float MultiPointCloudToPlaneError();
		void MultiViewRegistration(int noIterations);
		void Visualize();
		void VisualizeMergedViews();

	public:
		CRVLMem *pMem0;
		CRVLMem *pMem;
		CRVLParameterList paramList;
		DWORD mode;
		SurfelGraph *pSurfels;
		//SURFEL::ObjectGraph *pObjects;
#ifndef AR20				
		PlanarSurfelDetector *pSurfelDetector;
#endif
		Visualizer *pVisualizer;


		// new Mateja
		int minSurfelSize;
		int image_width;
		int image_height;
		int cell_size_width;
		int cell_size_height;
		int noColumnsImage;
		int noRowsImage;

		std::vector<View*> Views;
		Camera camera;

		ProjectionCube cube;
		Array<Point> centralPointsImage;

		std::vector<struct CrspSurfels*> crspSurfels;


		
		std::vector<struct CrspSurfelPairs*> crspPairs;

		std::vector<struct CrspSurfelPairs*> crspPairs_best;

		//std::vector<Pair<int, int>> associations;
		//std::vector<Pair<int, int>> CorrespondingSurfelPairs;
		std::vector<Pair<int, int>> TrueSurfelPairs;



		std::vector<struct CrspPose*> crspPoses;


		
		std::vector<struct crspPts> crspPts_list;



		std::vector<struct overlappingPoints> O_kl_list;
		

		std::vector<int> overlappingViews_list;


		std::vector<struct overlappingViews> overlappingViews_covariances;
		float rotErrorBound;
	};
}	// namespace RVL

