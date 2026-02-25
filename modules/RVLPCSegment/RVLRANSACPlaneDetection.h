//#pragma once
//class RVLRANSACPlaneDetection
//{
//public:
//	RVLRANSACPlaneDetection();
//	virtual ~RVLRANSACPlaneDetection();
//
//	//...................................................................................................................
//	//RANSAC - determining dominant plane
//	struct RV3DPOINT
//	{
//		int u, v, d, x, y, z;
//	};
//
//	BOOL Plane(RV3DPOINT *pPoint3D0, RV3DPOINT *pPoint3D1, RV3DPOINT *pPoint3D2, double &a, double &b, double &c);
//	void Consensus(RV3DPOINT *Point3DArray, int n3DPoints, double &a, double &b, double &c, RV3DPOINT **ConsensusSet, int &nConsensus);
//	void FindDominantPlane();
//	void Calculate_z(RV3DPOINT **BestConsensusSet, int nBest);
//
//	inline int RVRandom(int yMin, int yMax)
//	{
//		return  (rand() % yMax) + yMin;
//	}
//	//....................................................................................................................
//
//public:
//	double D; //for plane
//	double aBest, bBest, cBest, dBest; //plane coeff.
//	cv::Mat zRC;
//};
//
