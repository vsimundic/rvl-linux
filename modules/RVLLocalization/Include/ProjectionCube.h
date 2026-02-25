#pragma once

namespace RVL
{
	class ProjectionCube
	{
	public:
		ProjectionCube();
		virtual ~ProjectionCube();
		void Clear();

		float CosAngle;
		float distance_threshold;

		int cubeHalfSize;
		float RCube[6][9];
		int *cubeMatrix[6];
		Array<Point> transformedPoints;
		
		void CreateProjectionCube(int cubeHalfSizeIn, float CosAngle_in, float distance_threshold_in);
		void FindProjectionPosition(Point*, int*);
		
		void ProjectOnCube(Mesh*, float*, float*);
		void ProjectCentralPoints(Array<Point> centralPoints_array, float *R_cameraView, float *t_cameraView);
		//void ProjectOnCube(Mesh*, float*, float*, char*);
		
		void CorrespondenceCube();

		
	};
}


