#pragma once

namespace RVL
{
	// Move this structure to RVL3DTools.h.

	struct Plane
	{
		float N[3];
		float d;
	};

	//

	class ConvexHullCreator
	{
	public:
		ConvexHullCreator();
		virtual ~ConvexHullCreator();
		void Create(
			CRVLMem *pMemIn,
			int maxnPlanesIn);
		void Clear();
		void ConvexHull(
			Array<Plane> planesIn,
			Array<Vector3<float>> vertices,
			Mesh* pMesh);
		void DisplayConvexHull(
			Mesh *pMesh,
			Visualizer* pVisualizer);

	private:
		AccuSphere accuSphere;
		Array<Plane> planes;
		int maxnPlanes;
		int maxnEnclosingPlanes;
		float fAccuSphereCHRange;
		float accuSphereCHOffset;
		Mesh convexHullH;
		Array2D<float> planeData;
		CRVLMem* pMem;
	};
}

