//#include "stdafx.h"
//#include "RVLCore2.h"
//
//#include "Eigen\Dense"
//#include <Eigen\Eigenvalues>
//#include "opencv2/opencv.hpp"
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv/cv.h>
//#include "RVLAstra.h"
//#include "RVLRANSACPlaneDetection.h"
//
//
//RVLRANSACPlaneDetection::RVLRANSACPlaneDetection()
//{
//}
//
//
//RVLRANSACPlaneDetection::~RVLRANSACPlaneDetection()
//{
//}
//
//void RVLRANSACPlaneDetection::FindDominantPlane()
//{
//	cv::Mat pRGBImage;
//	cv::Mat pDepthImage;
//	int *DepthMap = 0;
//	int n3DPoints = 0;
//
//	int iWidth, iHeight;
//
//	RV3DPOINT *Point3DArray = 0;
//
//	//astra_temp_update();
//	//cv::flip(Image, ImageF, 1);
//	//cv::imshow("Color image", ImageF);
//
//	//memcpy(depth.data, imgD, 640 * 480 * sizeof(int16_t));
//	//double minVal, maxVal;
//	//cv::flip(depth, depthF, 1);
//	//cv::minMaxLoc(depthF, &minVal, &maxVal);
//	//depthF.convertTo(depthImg, CV_8U, -255.0f / maxVal, 255.0f);
//	//cv::imshow("Depth image", depthImg);
//	//cv::waitKey(2000);
//
//#ifdef RVL_ASTRA
//	Astra astra;
//	astra.GetImageFromAstra();
//#endif
//
//	iWidth = 640;
//	iHeight = 480;
//	DepthMap = new int[iWidth * iHeight];
//	memset(DepthMap, 0, iWidth * iHeight*sizeof(int));
//
//	Point3DArray = new RV3DPOINT[iWidth * iHeight];
//
//	memset(Point3DArray, 0, iWidth * iHeight * sizeof(RV3DPOINT));
//
//	int u, v;
//	int16_t d;
//	int dmin = 0;
//	int dmax = 0;
//
//	RV3DPOINT *pPoint3D = Point3DArray;
//	n3DPoints = 0;
//
//	//Determine max and min depth values and get Depth map
//	for (v = 0; v<480; v++)
//	{
//		for (u = 0; u<640; u++)
//		{
//			d = depthF.at<uint16_t>(v, u);
//			if (d == 0)
//			{
//				d = -1;
//			}
//			else
//			{
//				//count number of valid 3D points
//				n3DPoints++;
//
//				//determine min and max d
//				if (d<dmin)
//					dmin = d;
//
//				if (d>dmax)
//					dmax = d;
//			}
//
//			DepthMap[v * 640 + u] = d;
//
//			if (d != -1)
//			{
//				pPoint3D->u = u;
//				pPoint3D->v = v;
//				pPoint3D->d = d;
//
//				//gain x, y and z:
//				pPoint3D->z = d;
//				pPoint3D->x = (u - uc)*d / fu;
//				pPoint3D->y = (v - vc)*d / fv;
//
//				pPoint3D++;
//			}
//		}
//	}
//
//	////Form grayscale pic -> Scale from 1 to 255 (reserve 0 for undefined regions)
//	//for (v = 0; v<480; v++)
//	//{
//	//	for (u = 0; u<640; u++)
//	//	{
//	//		d = DepthMap[v * 640 + u];
//
//	//		if (d != -1)
//	//			d = ((d - dmin) * 254 / (dmax - dmin)) + 1;
//	//		else
//	//			d = 0;
//
//	//		depthF.at<uint16_t>(v,u) = d;
//	//	}
//	//}
//
//	pDepthImage = depthF;
//
//	double a, b, c;
//	int nConsensus;
//
//	int nBest = 0;
//
//
//	int iter;
//
//	int maxRANSACIter = 1000; //Max RANSAC iterations
//	//maxRANSACIter = 1;
//
//	RV3DPOINT  *pPoint3D0, *pPoint3D1, *pPoint3D2;
//
//	RV3DPOINT **Point3DPtrBuff1 = new RV3DPOINT *[n3DPoints];
//	RV3DPOINT **Point3DPtrBuff2 = new RV3DPOINT *[n3DPoints];
//
//	RV3DPOINT **ConsensusSet = Point3DPtrBuff1;
//	RV3DPOINT **BestConsensusSet = Point3DPtrBuff2;
//
//	RV3DPOINT **tmp;
//
//
//	int iR0, iR1, iR2;
//
//	//main RANSAC loop
//	for (iter = 0; iter<maxRANSACIter; iter++)
//	{
//		// randomly select 3 points from set A
//		iR0 = RVRandom(0, n3DPoints - 1);
//		iR1 = RVRandom(0, n3DPoints - 1);
//		iR2 = RVRandom(0, n3DPoints - 1);
//
//		pPoint3D0 = (Point3DArray + iR0);
//		pPoint3D1 = (Point3DArray + iR1);
//		pPoint3D2 = (Point3DArray + iR2);
//
//		//RV3DPOINT P0, P1, P2;
//
//		//if (iter == 0)
//		//{
//		//	P0.u = 175;
//		//	P0.v = 140;
//		//	P0.d = 1000000 / depthF.at<uint16_t>(P0.v, P0.u);
//		//	P1.u = 100;
//		//	P1.v = 390;
//		//	P1.d = 1000000 / depthF.at<uint16_t>(P1.v, P1.u);
//		//	P2.u = 540;
//		//	P2.v = 390;
//		//	P2.d = 1000000 / depthF.at<uint16_t>(P2.v, P2.u);
//		//	pPoint3D0 = &P0;
//		//	pPoint3D1 = &P1;
//		//	pPoint3D2 = &P2;
//		//}
//
//		if (!Plane(pPoint3D0, pPoint3D1, pPoint3D2, a, b, c))
//			continue;
//
//		Consensus(Point3DArray, n3DPoints, a, b, c, ConsensusSet, nConsensus);
//
//		//Test best values and consensus set
//		if (nConsensus > nBest)
//		{
//			nBest = nConsensus;
//
//			aBest = a;
//			bBest = b;
//			cBest = c;
//			dBest = D;
//
//			tmp = ConsensusSet;
//			ConsensusSet = BestConsensusSet;
//			BestConsensusSet = tmp;
//		}
//
//	}
//
//	Calculate_z(BestConsensusSet, nBest);
//
//	//D = dBest;
//
//	//Consensus(Point3DArray, n3DPoints, aBest, bBest, cBest, BestConsensusSet, nBest);
//
//	cv::Mat pColor;
//
//	FILE *fp;
//	fp = fopen("normal.txt", "w");
//	fprintf(fp, "%f\t%f\t%f\n", zRC.at<float>(0), zRC.at<float>(1), zRC.at<float>(2));
//	fclose(fp);
//	float x0, y0, z0, dist;
//	dist = dBest;
//
//	fp = fopen("d.txt", "w");
//	fprintf(fp, "%f\n", dist);
//	fclose(fp);
//
//	fp = fopen("dominantPlane.txt", "w");
//	for (int i = 0; i<nBest; i++)
//	{
//		pPoint3D = BestConsensusSet[i];
//
//		x0 = pPoint3D->x;
//		y0 = pPoint3D->y;
//		z0 = pPoint3D->z;
//		//dist = -a*x0 - b*y0 - c*z0;
//		//fprintf(fp, "%f\t%f\t%f\t%f\n", x0, y0, z0, dist);
//		fprintf(fp, "%f\t%f\t%f\n", x0, y0, z0);
//
//		//for image show:
//		ImageF.at<cv::Vec3b>(pPoint3D->v, pPoint3D->u)[0] = 0;
//		ImageF.at<cv::Vec3b>(pPoint3D->v, pPoint3D->u)[1] = 255;
//		ImageF.at<cv::Vec3b>(pPoint3D->v, pPoint3D->u)[2] = 0;
//	}
//	fclose(fp);
//	cv::imshow("Dominant Plane", ImageF);
//	cv::waitKey();
//
//	//exit elegantly 
//	delete[] DepthMap;
//	delete[] Point3DArray;
//	delete[] Point3DPtrBuff1;
//	delete[] Point3DPtrBuff2;
//}
//
//
///*****************************HELPER FUNCTIONS RANSAC*************************************/
//BOOL RVLRANSACPlaneDetection::Plane(RV3DPOINT *pPoint3D0, RV3DPOINT *pPoint3D1, RV3DPOINT *pPoint3D2, double &a, double &b, double &c)
//{
//	int xa, xb, xc, ya, yb, yc, za, zb, zc;
//
//
//	xa = pPoint3D0->x;
//	ya = pPoint3D0->y;
//	za = pPoint3D0->z;
//
//	xb = pPoint3D1->x;
//	yb = pPoint3D1->y;
//	zb = pPoint3D1->z;
//
//	xc = pPoint3D2->x;
//	yc = pPoint3D2->y;
//	zc = pPoint3D2->z;
//
//	if (sqrt((xb - xa)*(xb - xa) + (yb - ya)*(yb - ya) + (zb - za)*(zb - za)) < 100 || sqrt((xc - xa)*(xc - xa) + (yc - ya)*(yc - ya) + (zc - za)*(zc - za)) < 100)
//		return FALSE;
//
//	else
//	{
//		a = ((zc - za)*(yb - ya)) - ((yc - ya)*(zb - za));
//		b = ((zb - za)*(xc - xa)) - ((zc - za)*(xb - xa));
//		c = ((xb - xa)*(yc - ya)) - ((xc - xa)*(yb - ya));
//
//		if (((xb - xa)*(xc - xa) + (yb - ya)*(yc - ya) + (zb - za)*(zc - za)) / (sqrt((xb - xa)*(xb - xa) + (yb - ya)*(yb - ya) + (zb - za)*(zb - za))* sqrt((xc - xa)*(xc - xa) + (yc - ya)*(yc - ya) + (zc - za)*(zc - za))) > 0.866)
//			return FALSE;
//
//		else
//		{
//			if (c > 0)
//			{
//				a *= -1;
//				b *= -1;
//				c *= -1;
//			}
//
//			double norm = sqrt(a*a + b*b + c*c);
//
//			a /= norm;
//			b /= norm;
//			c /= norm;
//
//			D = pPoint3D0->x*a + pPoint3D0->y*b + pPoint3D0->z*c;
//
//			return TRUE;
//
//		}
//
//	}
//	//#define cvReleaseMatHeader cvReleaseMat
//	//double A[3 * 3];
//
//	//A[0 * 3 + 0] = pPoint3D0->u;
//	//A[0 * 3 + 1] = pPoint3D0->v;
//	//A[0 * 3 + 2] = 1.0;
//
//	//A[1 * 3 + 0] = pPoint3D1->u;
//	//A[1 * 3 + 1] = pPoint3D1->v;
//	//A[1 * 3 + 2] = 1.0;
//
//	//A[2 * 3 + 0] = pPoint3D2->u;
//	//A[2 * 3 + 1] = pPoint3D2->v;
//	//A[2 * 3 + 2] = 1.0;
//
//	//double Z[3];
//
//	//Z[0] = 1000000.0 / pPoint3D0->d;
//	//Z[1] = 1000000.0 / pPoint3D1->d;
//	//Z[2] = 1000000.0 / pPoint3D2->d;
//
//	//CvMat *A_ = cvCreateMatHeader(3, 3, CV_64FC1);
//	//A_->data.db = A;
//	//CvMat *Z_ = cvCreateMatHeader(3, 1, CV_64FC1);
//	//Z_->data.db = Z;
//
//	//double p[3];
//	//CvMat *p_ = cvCreateMatHeader(3, 1, CV_64FC1);
//	//p_->data.db = p;
//
//	//if (cvSolve(A_, Z_, p_))
//	//{
//	//	a = p[0];
//	//	b = p[1];
//	//	c = p[2];
//
//	//	cvReleaseMatHeader(&A_);
//	//	cvReleaseMatHeader(&Z_);
//	//	cvReleaseMatHeader(&p_);
//
//	//	return TRUE;
//	//}
//	//else
//	//{
//	//	cvReleaseMatHeader(&A_);
//	//	cvReleaseMatHeader(&Z_);
//	//	cvReleaseMatHeader(&p_);
//
//	//	return FALSE;
//	//}
//}
//
//void RVLRANSACPlaneDetection::Consensus(RV3DPOINT *Point3DArray, int n3DPoints, double &a, double &b, double &c, RV3DPOINT **ConsensusSet, int &nConsensus)
//{
//	RV3DPOINT *pPoint3D = Point3DArray;
//
//	RV3DPOINT **pConsensusSet = ConsensusSet;
//	double Tol = 5.0;
//	//double Tol = 0.01;
//
//	int i;
//	double e;
//
//	nConsensus = 0;
//
//	for (i = 0; i < n3DPoints; i++, pPoint3D++)
//	{
//		e = ((pPoint3D->x * a) + (pPoint3D->y * b) + (pPoint3D->z*c)) - D;
//		if (e > -Tol && e < Tol)
//			*(pConsensusSet++) = pPoint3D;
//		else
//			continue;
//
//		//e = 1000000.0 / (double)(pPoint3D->d) - (a * pPoint3D->u + b * pPoint3D->v + c);
//
//		//if (e > Tol)
//		//	continue;
//
//		//if (e >= -Tol)
//		//	*(pConsensusSet++) = pPoint3D;
//
//
//	}
//
//	nConsensus = pConsensusSet - ConsensusSet;
//
//}
///*********************************************************************************/
//
//void RVLRANSACPlaneDetection::Calculate_z(RV3DPOINT **BestConsensusSet, int nBest)
//{
//	Moments<double> moments;
//
//	InitMoments<double>(moments);
//
//	RV3DPOINT *pPoint3D;
//
//	int i;
//	double P[3];
//
//	for (i = 0; i < nBest; i++)
//	{
//		pPoint3D = BestConsensusSet[i];
//
//		P[0] = pPoint3D->x;
//		P[1] = pPoint3D->y;
//		P[2] = pPoint3D->z;
//		UpdateMoments<double>(moments, P);
//	}
//
//	double C[9];
//	double t[3];
//
//	GetCovMatrix3<double>(&moments, C, t);
//
//	//Eigen::EigenSolver<Eigen::Matrix3f> eigenSolver(Eigen::Map<Eigen::Matrix3f>(C));
//	Eigen::EigenSolver<Eigen::Matrix3d> eigenSolver;
//
//	eigenSolver.compute(Eigen::Map<Eigen::Matrix3d>(C));
//
//	Eigen::Matrix3d R = eigenSolver.pseudoEigenvectors();
//
//	if (R(2, 2) > 0)
//	{
//		zRC.at<float>(0) = -1 * R(0, 2);
//		zRC.at<float>(1) = -1 * R(1, 2);
//		zRC.at<float>(2) = -1 * R(2, 2);
//	}
//	else
//	{
//		zRC.at<float>(0) = R(0, 2);
//		zRC.at<float>(1) = R(1, 2);
//		zRC.at<float>(2) = R(2, 2);
//	}
//
//
//	//distribution.R[0] = R(0, 0);
//	//distribution.R[1] = R(1, 0);
//	//distribution.R[2] = R(2, 0);
//	//distribution.R[3] = R(0, 1);
//	//distribution.R[4] = R(1, 1);
//	//distribution.R[5] = R(2, 1);
//	//distribution.R[6] = R(0, 2);
//	//distribution.R[7] = R(1, 2);
//	//distribution.R[8] = R(2, 2);
//
//	////Eigen::Map<Eigen::Vector3f>(distribution.var) = eigenSolver.eigenvalues();
//
//	//Eigen::Vector3cd var_ = eigenSolver.eigenvalues();
//
//	//distribution.var[0] = var_[0].real();
//	//distribution.var[1] = var_[1].real();
//	//distribution.var[2] = var_[2].real();
//}