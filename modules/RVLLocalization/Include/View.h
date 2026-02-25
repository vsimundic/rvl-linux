#pragma once

//Header file ==> Function Declarations 

namespace RVL
{
	class CameraView
	{
	public:


	public:
		int iCameraView;
		float R[9];
		float t[3];
		float x[3];
		float y[3];
		float z[3];
		float T[16];

		int LoadCameraFile(char *filePath);

	private:

	};
	
	class View
	{
	public:
		//function declarations
		View(); //default contructor
		virtual ~View(); //destructor
		void Clear();

		//Overload Constructors
		//void Create(char *fileName, Mesh *pMesh, SurfelGraph * pSurfels, CameraView *pCameraView); // SUNCG -v1
		void Create(char *fileName, Mesh *pMesh, SurfelGraph * pSurfels, CameraView cameraView);
		void Load(Mesh *pMesh);
		void InformationContnet();
	
		Mesh mesh;
		SurfelGraph surfels;
		ProjectionCube projectionCube;
		CameraView* pCameraView;
		CameraView cameraView;	
		char *fileName;

		// Image properties
		int image_width;
		int image_height;

		// Sample image to cells
		// Set size of one cell
		int image_cell_size_width;
		int image_cell_size_height;

		int noColumnsImage;
		int noRowsImage;

		//struct imageCell
		//{
		//	int row;
		//	int column;
		//	float covariance[9];
		//};

		float *covariance_list;

		Array<Point> centralPoints_array;

		struct surfelPair
		{
			int S1_ID;
			int S2_ID;

		};
		std::vector<struct surfelPair*> surfelPairs_list;
		
		// Multiview
		std::vector<int> overlapingViewList;

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

		std::vector<struct overlappingPoints> O_kl_list;

		std::vector<struct overlappingPoints> C_kl_list;				// sampled points

		std::vector<std::vector<struct overlappingPoints>> O_k_list;

		std::vector<std::vector<struct overlappingPoints>> C_k_list;	// sampled points

		std::vector<int> overlappingViews_list;

		struct overlappingViews
		{
			int jView;
			float covariance_ij[9];
			float c_ij;
		};

		std::vector<struct overlappingViews> overlappingViews_covariances;

		std::vector<Pair<int, int>> correspondences_iView_jView;
		

		std::vector<std::vector<Pair<int, int>>> correspodance_list;

	private:
		//member variables
	};

	



	namespace LOCAL
	{
		int LoadViews(char *viewFileName, std::vector<CameraView>* cameraViews);
	}
}

