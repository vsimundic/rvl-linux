#ifdef RVLHDF5
#include "H5Cpp.h"
#endif
#ifdef RVLLINUX
#define RVLFILEPATH_SEPARATOR '/'
#define RVLFILEPATH_SEPARATOR_ "/"
#else
#define RVLFILEPATH_SEPARATOR '\\'
#define RVLFILEPATH_SEPARATOR_ "\\"
#endif

#define RVL_DELETE_ARRAY(Array) \
	{                           \
		if (Array)              \
			delete[] Array;     \
		Array = NULL;           \
	}
#define RVLSCALECOLOR(SrcColor, a, TgtColor)                         \
	{                                                                \
		TgtColor[0] = (unsigned char)((int)(SrcColor[0]) * a / 100); \
		TgtColor[1] = (unsigned char)((int)(SrcColor[1]) * a / 100); \
		TgtColor[2] = (unsigned char)((int)(SrcColor[2]) * a / 100); \
	}
#define RVLSCALECOLOR2(SrcColor, scale, TgtColor)                           \
	{                                                                       \
		TgtColor[0] = (unsigned char)((int)(SrcColor[0]) * scale[0] / 100); \
		TgtColor[1] = (unsigned char)((int)(SrcColor[1]) * scale[1] / 100); \
		TgtColor[2] = (unsigned char)((int)(SrcColor[2]) * scale[2] / 100); \
	}
// X = 0(2x1)
#define RVLNULL2VECTOR(X)  \
	{                      \
		X[0] = X[1] = 0.0; \
	}
// Tgt = -Src(2x1)
#define RVLNEGVECT2(Src, Tgt) \
	{                         \
		Tgt[0] = -Src[0];     \
		Tgt[1] = -Src[1];     \
	}
// normalize vector x(2x1)
#define RVLNORM2(x, len)                  \
	{                                     \
		len = sqrt(RVLDOTPRODUCT2(x, x)); \
		RVLSCALE2VECTOR2(x, len, x);      \
	}
// Copy X to Y.
#define RVLCOPY2VECTOR(X, Y) \
	{                        \
		Y[0] = X[0];         \
		Y[1] = X[1];         \
	}
// Z = X + Y
#define RVLSUM2VECTORS(X, Y, Z) \
	{                           \
		Z[0] = X[0] + Y[0];     \
		Z[1] = X[1] + Y[1];     \
	}
// Z = X - Y
#define RVLDIF2VECTORS(X, Y, Z) \
	{                           \
		Z[0] = X[0] - Y[0];     \
		Z[1] = X[1] - Y[1];     \
	}
// Y = s * X
#define RVLSCALE2VECTOR(X, s, Y) \
	{                            \
		Y[0] = s * X[0];         \
		Y[1] = s * X[1];         \
	}
// Y = 1/s * X
#define RVLSCALE2VECTOR2(X, s, Y) \
	{                             \
		Y[0] = X[0] / s;          \
		Y[1] = X[1] / s;          \
	}
// Dot product of X and Y.
#define RVLDOTPRODUCT2(X, Y) (X[0] * Y[0] + X[1] * Y[1])
// y = A(2x2) * x(2x1)
#define RVLMULMX2X2VECT(A, x, y)          \
	{                                     \
		y[0] = A[0] * x[0] + A[1] * x[1]; \
		y[1] = A[2] * x[0] + A[3] * x[1]; \
	}
// y = A(2x2)' * x(2x1)
#define RVLMULMX2X2TVECT(A, x, y)         \
	{                                     \
		y[0] = A[0] * x[0] + A[2] * x[1]; \
		y[1] = A[1] * x[0] + A[3] * x[1]; \
	}
// R = [cs, -sn;
//		sn,  cs]
#define RVLROT2D(cs, sn, R) \
	{                       \
		R[0] = cs;          \
		R[1] = -sn;         \
		R[2] = sn;          \
		R[3] = cs;          \
	};

#define RVLGETFILEEXTENSION(FileName) (strrchr(FileName, '.') + 1)
#define RVLCROPRECT(minx, maxx, miny, maxy, left, right, top, bottom) \
	{                                                                 \
		if (top < miny)                                               \
			top = miny;                                               \
		if (bottom > maxy)                                            \
			bottom = maxy;                                            \
		if (left < minx)                                              \
			left = minx;                                              \
		if (right > maxx)                                             \
			right = maxx;                                             \
	}
#define RVLNEIGHBORHOOD(x0, y0, halfNeighborhoodSize, minx, maxx, miny, maxy, left, right, top, bottom) \
	{                                                                                                   \
		left = x0 - halfNeighborhoodSize;                                                               \
		right = x0 + halfNeighborhoodSize;                                                              \
		top = y0 - halfNeighborhoodSize;                                                                \
		bottom = y0 + halfNeighborhoodSize;                                                             \
		RVLCROPRECT(minx, maxx, miny, maxy, left, right, top, bottom)                                   \
	}
#define RVLRND(n, iRnd, nRnd, iiRnd, x) \
	{                                   \
		x = iRnd[iiRnd] % n;            \
		iiRnd = (iiRnd + 1) % nRnd;     \
	}
// y = s * x
#define RVLSCALEVECTOR(x, s, y, n, i) \
	{                                 \
		for (i = 0; i < n; i++)       \
			y[i] = s * x[i];          \
	}
// c = a + b;
#define RVLSUMVECTORS(a, b, n, c, i) \
	{                                \
		for (i = 0; i < n; i++)      \
			c[i] = a[i] + b[i];      \
	}
// c = a - b;
#define RVLDIFVECTORS(a, b, n, c, i) \
	{                                \
		for (i = 0; i < n; i++)      \
			c[i] = a[i] - b[i];      \
	}
// c = a' * b
#define RVLDOTPRODUCT(a, b, n, c, i) \
	{                                \
		c = 0;                       \
		for (i = 0; i < n; i++)      \
			c += a[i] * b[i];        \
	}
// c = A(mxn) * b
#define RVLMULMXVECT(A, b, m, n, c, i, j, a) \
	{                                        \
		for (j = 0; j < m; j++)              \
		{                                    \
			a = A + j * n;                   \
			RVLDOTPRODUCT(a, b, n, c[j], i)  \
		}                                    \
	};
// c = A(mxn)' * b
#define RVLMULMXTVECT(A, b, m, n, c, i, j, a) \
	{                                         \
		for (j = 0; j < n; j++)               \
		{                                     \
			c[j] = 0;                         \
			a = A + j;                        \
			for (i = 0; i < m; i++)           \
				c[j] += (a[i * n] * b[i]);    \
		}                                     \
	}
// Z(n1 x n3) = A(n1 x n2) * B(n2 * n3)
#define RVLMXMUL(A, B, Z, n1, n2, n3)                   \
	{                                                   \
		int i, j, k;                                    \
		double z;                                       \
		for (i = 0; i < n1; i++)                        \
			for (j = 0; j < n3; j++)                    \
			{                                           \
				z = 0.0;                                \
				for (k = 0; k < n2; k++)                \
					z += A[i * n2 + k] * B[k * n3 + j]; \
				Z[i * n3 + j] = z;                      \
			}                                           \
	}
// Normalize angle x to interval <-PI, PI]
#define RVLNORMANGLE(x) x = (x > PI ? x -= (2.0 * PI) : (x <= -PI ? x += (2.0 * PI) : x))
// Bresenham line drawing algorithm - initialization
#define RVLBRESENHAMINIT(x0, y0, x1, y1, x, y, data) \
	{                                                \
		data.xEnd = x1;                              \
		data.yEnd = y1;                              \
		data.dx = x1 - x0;                           \
		data.dy = y1 - y0;                           \
		data.absdx = RVLABS(data.dx);                \
		data.absdy = RVLABS(data.dy);                \
		if (RVLABS(data.dy) < RVLABS(data.dx))       \
		{                                            \
			data.xi = 0;                             \
			data.yi = (data.dy > 0 ? 1 : -1);        \
			data.xii = (data.dx > 0 ? 1 : -1);       \
			data.yii = 0;                            \
			data.dD1 = data.absdx;                   \
			data.dD2 = data.absdy;                   \
		}                                            \
		else                                         \
		{                                            \
			data.xi = (data.dx > 0 ? 1 : -1);        \
			data.yi = 0;                             \
			data.xii = 0;                            \
			data.yii = (data.dy > 0 ? 1 : -1);       \
			data.dD1 = data.absdy;                   \
			data.dD2 = data.absdx;                   \
		}                                            \
		data.D = 2 * data.dD2 - data.dD1;            \
		x = x0;                                      \
		y = y0;                                      \
		data.bCompleted = false;                     \
	}
// Bresenham line drawing algorithm - update
#define RVLBRESENHAMUPDATE(data, x, y)                        \
	{                                                         \
		if (data.D > 0)                                       \
		{                                                     \
			x += data.xi;                                     \
			y += data.yi;                                     \
			data.D -= (2 * data.dD1);                         \
		}                                                     \
		data.D += (2 * data.dD2);                             \
		x += data.xii;                                        \
		y += data.yii;                                        \
		data.bCompleted = (x == data.xEnd && y == data.yEnd); \
	}
// This is how you use the Bresenham macros:
//
// int x1, y1, x2, y2; // Input: line endpoint coordinates
// BresenhamData bresenhamData;
// int x, y;
// RVLBRESENHAMINIT(x1, y1, x2, y2, x, y, bresenhamData);
// while (true)
//{
//	// Do whatever you want with (x, y).
//	if (bresenhamData.bCompleted)
//		break;
//	RVLBRESENHAMUPDATE(bresenhamData, x, y);
//}
// Given a unit vector uS and rotation matrix R, determine the closest convex template element.
#define RVLGET_CLOSEST_CONVEX_TEMPLATE_ELEMENT(convexTemplateLUT, uS, R, uM, i, j, k, iCorrespondence)             \
	{                                                                                                              \
		if (R)                                                                                                     \
		{                                                                                                          \
			RVLMULMX3X3TVECT(R, uS, uM)                                                                            \
		}                                                                                                          \
		else                                                                                                       \
		{                                                                                                          \
			RVLCOPY3VECTOR(uS, uM);                                                                                \
		}                                                                                                          \
		i = (int)(uM[0] / convexTemplateLUT.quant + convexTemplateLUT.halfRange);                                  \
		j = (int)(uM[1] / convexTemplateLUT.quant + convexTemplateLUT.halfRange);                                  \
		k = (int)(uM[2] / convexTemplateLUT.quant + convexTemplateLUT.halfRange);                                  \
		iCorrespondence = convexTemplateLUT.LUT.Element[RVL3DARRAY_ELEMENT_INDEX(convexTemplateLUT.LUT, i, j, k)]; \
	}

#define RVL_DATASET_FLAG_TUW_KINECT 0
#define RVL_DATASET_FLAG_WILLOW_AND_CHALLENGE 1
#define RVL_DATASET_FLAG_ICL 2

namespace RVL
{
	template <typename T>
	struct SortIndex
	{
		int idx;
		T cost;
	};

	template <typename T1, typename T2>
	struct Pair
	{
		T1 a;
		T2 b;
	};

	struct Point2D
	{
		float P[2];
	};

	template <typename Type>
	struct Rect
	{
		Type minx;
		Type maxx;
		Type miny;
		Type maxy;
	};

	struct Camera
	{
		float fu;
		float fv;
		float uc;
		float vc;
		int w;
		int h;
	};

	struct Polygon2DLineIntersection
	{
		int iVertex;
		Point2D P;
		char dir;
	};

	// VIDOVIC
	struct GTInstance
	{
		int iScene;
		int iModel;
		float R[9];
		float t[3];
		bool matched;
	};

	struct SegmentGTInstance
	{
		int iScene;
		int iSSegment;
		int iModel;
		int iMSegment;
		int matchID;
		bool valid;
		// unsigned char color[3];
		SegmentGTInstance *pNext;
	};

	struct ModelColor
	{
		int iModel;
		unsigned char color[3];
		ModelColor *pNext;
	};

	struct BresenhamData
	{
		int xEnd;
		int yEnd;
		int dx;
		int dy;
		int absdx;
		int absdy;
		int D;
		int xi;
		int yi;
		int xii;
		int yii;
		int dD1;
		int dD2;
		bool bCompleted;
	};

	struct ConvexTemplateLookUpTable
	{
		float quant;
		int iHalfRange;
		float halfRange;
		Array3D<int> LUT;
	};

	bool GetAngleAxis(float *R, float *V, float &theta);
	void GetDistance(float *t, float &distance);
	// END VIDOVIC

	void PrintMatrix(FILE *fp, double *A, int n, int m);
	void QuickSort(int *Key, int *Index, int n);
	void RandomColor(unsigned char *color);
	void RandomColors(
		unsigned char *SelectionColor,
		unsigned char *&colorArray,
		int n);
	void RandomIndices(Array<int> &A);
	void RandomIndices(
		Array<int> rnd,
		int iRnd,
		Array<int> &A);
	void Permute(
		Array<int> rnd,
		int iRnd,
		Array<int> A);
	void Combinations(
		int n,
		int m,
		Array2D<int> &comb,
		int *idxMem = NULL);
	void Combinations(
		Array<int> N,
		Array2D<int> &V);
	void DistributionFromCovMx(
		float *C,
		float *R,
		float *var);
	char *ReadConfigurationFile(char *cfgSelectionFileName);
	void MeanShift1D(
		Array<int> histogram,
		int winSize,
		int minClusterSize,
		Array<int> &clusters,
		int *assignment,
		int *wAgg,
		int *move = NULL,
		int *w = NULL);
	void CreateImage3x3NeighborhoodLT(
		int w,
		int *neighbor);
	void CreateGrayScaleImage(Array2D<float> inputImage, cv::Mat &outputImage);
	void SetCameraParams(Camera &camera, float fu, float fv, float uc, float vc, int w, int h);
	void MinBoundingBoxF(
		Array<Point2D> poly,
		float *C,
		float &a,
		float &b,
		float &ca,
		float &sa);
	bool MinBoundingBox(
		Array<Point2D> poly,
		float *C,
		float &a,
		float &b,
		float &ca,
		float &sa,
		float &area);
	void ReadLine(
		FILE *fp,
		int nCharacters,
		char *line);
	int FurthestPoint(
		float *P,
		Array<Vector3<float>> points,
		float *V = NULL);
	void CreateConvexTemplate6(float *A);
	void CreateConvexTemplate18(float *A);
	void CreateConvexTemplate66(float *A);
	void CreateTemplateLookUpTable(
		Array2D<float> A,
		int resolution,
		ConvexTemplateLookUpTable &CTLUT);
	void RotatedRR(
		float *RSrc,
		float *q,
		float *RTgt);
	void RotateRdR(
		float *RSrc,
		float *q,
		float *RTgt);
	void Move(
		float *x,
		Pose3D *pPoseSrc,
		Pose3D *pPoseTgt);
	void Move(
		Pose3D *pPoseSrc,
		float *x,
		Pose3D *pPoseTgt);
	void PseudoRndSampleUnitSphere(
		float *x,
		Array<int> rndVal,
		int &iRndVal);
	void IntrinsicCameraMatrix(
		Camera camera,
		float *K);
	void InvIntrinsicCameraMatrix(
		float *K,
		float *invK);
	void TriangleNormalAndArea(
		float **P,
		float *N,
		float &area);
	bool SortCompare(SortIndex<float> x1, SortIndex<float> x2);

	// created by Damir Filko
	// adapted for general case by Robert Cupec

	template <class Type>
	void BubbleSort(Array<Type> &InOutArray,
					bool descending = false)
	{
		Type tempVoid;
		bool chg = true;

		int i;

		while (chg)
		{
			chg = false;
			for (i = 0; i < InOutArray.n - 1; i++)
			{
				if (descending)
				{
					if (InOutArray.Element[i + 1].cost > InOutArray.Element[i].cost)
					{
						tempVoid = InOutArray.Element[i];
						InOutArray.Element[i] = InOutArray.Element[i + 1];
						InOutArray.Element[i + 1] = tempVoid;

						chg = true;
					}
				}
				else
				{
					if (InOutArray.Element[i + 1].cost < InOutArray.Element[i].cost)
					{
						tempVoid = InOutArray.Element[i];
						InOutArray.Element[i] = InOutArray.Element[i + 1];
						InOutArray.Element[i + 1] = tempVoid;

						chg = true;
					}
				}
			}
		}
	}

	//

	template <class Type>
	bool Roots2(Type *p, Type *z)
	{
		Type det = p[1] * p[1] - 4.0 * p[0] * p[2];

		if (det < 0.0)
			return false;

		Type fTmp1 = 2.0 * p[2];
		Type fTmp2 = -p[1] / fTmp1;
		Type fTmp3 = sqrt(det) / fTmp1;

		z[0] = fTmp2 - fTmp3;
		z[1] = fTmp2 + fTmp3;

		return true;
	}

	template <class Type>
	bool Eig2(Type *C, Type *eig)
	{
		Type p[3];

		p[2] = 1.0;
		p[1] = -(C[0] + C[3]);
		p[0] = C[0] * C[3] - C[1] * C[2];

		return RVL::Roots2<Type>(p, eig);
	}

	template <typename T>
	void InitRect(Rect<T> *pRect, T *P)
	{
		pRect->minx = pRect->maxx = P[0];
		pRect->miny = pRect->maxy = P[1];
	}

	template <typename T>
	void UpdateRect(Rect<T> *pRect, T *P)
	{
		if (P[0] < pRect->minx)
			pRect->minx = P[0];
		else if (P[0] > pRect->maxx)
			pRect->maxx = P[0];

		if (P[1] < pRect->miny)
			pRect->miny = P[1];
		else if (P[1] > pRect->maxy)
			pRect->maxy = P[1];
	}

	template <typename T>
	bool IsInRect(T x, T y, Rect<T> rect)
	{
		if (x < rect.minx)
			return false;

		if (x > rect.maxx)
			return false;

		if (y < rect.miny)
			return false;

		if (y > rect.maxy)
			return false;

		return true;
	}

	template <typename T>
	bool IsContainedInRect(Rect<T> rectQuery, Rect<T> rect)
	{
		if (rectQuery.minx < rect.minx)
			return false;

		if (rectQuery.maxx > rect.maxx)
			return false;

		if (rectQuery.miny < rect.miny)
			return false;

		if (rectQuery.maxy > rect.maxy)
			return false;

		return true;
	}

	template <typename T>
	void ExpandRect(Rect<T> *pRect, T extension)
	{
		pRect->minx -= extension;
		pRect->maxx += extension;
		pRect->miny -= extension;
		pRect->maxy += extension;
	}

	template <typename T>
	void CropRect(
		Rect<T> &rect,
		Rect<T> cropWin)
	{
		if (rect.minx < cropWin.minx)
			rect.minx = cropWin.minx;
		if (rect.maxx > cropWin.maxx)
			rect.maxx = cropWin.maxx;
		if (rect.miny < cropWin.miny)
			rect.miny = cropWin.miny;
		if (rect.maxy > cropWin.maxy)
			rect.maxy = cropWin.maxy;
	}

	template <typename T>
	void SampleRect(
		Rect<T> *pRect,
		float border,
		Rect<T> cropWin,
		int nSamplesPerMaxSide,
		Array2D<T> &PtArray,
		Pair<int, int> **pPtIdxArray = NULL,
		int *pw = NULL,
		int *ph = NULL)
	{
		ExpandRect<T>(pRect, border);

		CropRect<T>(*pRect, cropWin);

		T w = pRect->maxx - pRect->minx;
		T h = pRect->maxy - pRect->miny;

		float u0 = (float)(pRect->minx);
		float v0 = (float)(pRect->miny);
		float fw = (float)w;
		float fh = (float)h;

		T a = RVLMAX(w, h);

		float da = (float)a / (float)(nSamplesPerMaxSide - 1);

		float fnw = round(fw / da);
		float dw = fw / fnw;
		int nw = (int)fnw;
		float fnh = round(fh / da);
		float dh = fh / fnh;
		int nh = (int)fnh;

		PtArray.w = 2;
		PtArray.h = (nh + 1) * (nw + 1);

		if (pw)
			*pw = nw + 1;

		if (ph)
			*ph = nh + 1;

		PtArray.Element = new T[PtArray.w * PtArray.h];

		if (pPtIdxArray)
			*pPtIdxArray = new Pair<int, int>[PtArray.w * PtArray.h];

		int iPt = 0;

		int i, j;
		float *m;
		Pair<int, int> *pPtIdx;

		for (i = 0; i <= nh; i++)
			for (j = 0; j <= nw; j++, iPt++)
			{
				m = PtArray.Element + PtArray.w * iPt;

				m[0] = u0 + (float)j * dw;
				m[1] = v0 + (float)i * dh;

				if (pPtIdxArray)
				{
					pPtIdx = (*pPtIdxArray) + iPt;

					pPtIdx->a = j;
					pPtIdx->b = i;
				}
			}
	}

	void GetFileNameAndPath(
		char *fileNameWithPath,
		char *&fileName,
		char *&filePath);
	void LoadCameraParametersFromFile(
		char *cfgFileName,
		Camera &camera,
		CRVLMem *pMem);

	// VIDOVIC
	class FileSequenceLoader
	{
	public:
		FileSequenceLoader();
		~FileSequenceLoader();

		bool Init(char *sequenceFileName);
		bool Get(int index, char *filePath, char *fileName, int *ID);
		bool GetNext(char *filePath, char *fileName, int *ID);
		bool Get(int index, char *filePath, char *fileName);
		bool GetNext(char *filePath, char *fileName);
		bool GetFilePath(int index, char *filePath);
		bool GetNextPath(char *filePath);
		bool GetFileName(int index, char *fileName);
		bool GetNextName(char *fileName);
		bool GetID(int index, int *ID);
		bool GetNextID(int *ID);
		int GetLastModelID();
		void AddModel(int ID, char *filePath, char *fileName);
		void ResetID();

	public:
		int nFileNames;

	private:
		std::vector<std::vector<char>> names;
		std::vector<std::vector<char>> paths;
		std::vector<int> IDs;
		int currentID;
	};

	class ECCVGTLoader
	{
	public:
		ECCVGTLoader();
		~ECCVGTLoader();

		bool Init(char *filePath, char *GTFolderPath, char *modelsID, DWORD dataSet = NULL);
		bool Init(FileSequenceLoader sceneSequence, char *GTFolderPath, char *modelsID, DWORD dataSet = NULL);
		bool SaveGTFile(char *filePath);
		void ResetMatchFlag();
		void LoadRotInvarianceInfo(char *rotInvarianceFileName);

	private:
		bool LoadModels(char *filePath);
		int FindModelID(char *modelName);
		void CreateGTFilePath(char *scenePath, char *GTFilePath);
		void CreateWillowAndChallengeGTFilePath(char *scenePath, char *GTFilePath);

	public:
		int nScenes;
		int nModels;
		int nModelsInDB;
		Array<Array<GTInstance>> GT;
		bool *bRotInvariantModel;
		DWORD dataSetFlag;

	private:
		int iScene;
		char *modelsInDB;
		char *GTFolder;
		char *GTFilePath;
	};

	void TransformWillowAndChallengeGT2ECCVFormat(char *sceneGTPath);
	void TransformICLGT2ECCVFormat(char *sceneGTPath);
	void TransformCorrectedICLGT2ECCVFormat(char *sceneGTPath);
	// END VIDOVIC

	template <typename T>
	struct QList2Array // Move to RVLQList.h
	{
		Array<QList<T>> listArray;
		T *mem;
	};

	template <class DataType, class CostType>
	void Min(Array<DataType> &InArray,
			 int nOut,
			 Array<DataType> &OutArray)
	{
		if (InArray.n <= 0 || nOut <= 0)
			return;

		OutArray.n = 0;

		int nBins = InArray.n / nOut + 1;

		QList2Array<QLIST::Index2> binArray[2];
		int *n[2];

		int i;

		for (i = 0; i < 2; i++)
		{
			binArray[i].listArray.Element = new QList<QLIST::Index2>[nBins];
			binArray[i].mem = new QLIST::Index2[InArray.n];
			n[i] = new int[nBins];
		}

		QList<QLIST::Index2> *bin = binArray[0].listArray.Element;

		RVLQLIST_INIT(bin);

		QLIST::Index2 *pIdx = binArray[0].mem;

		n[0][0] = 0;

		for (i = 0; i < InArray.n; i++)
		{
			pIdx->Idx = i;

			RVLQLIST_ADD_ENTRY(bin, pIdx);

			pIdx++;

			n[0][0]++;
		}

		int nBins_ = nBins;

		int iSrc = 0;
		int iSrcBin = 0;

		int iTgt = 1;

		CostType min, max;
		int idx, iTmp;
		float cost;

		while (OutArray.n < nOut)
		{
			if (OutArray.n == nOut - 1)
			{
				pIdx = binArray[iSrc].listArray.Element[iSrcBin].pFirst;

				min = InArray.Element[pIdx->Idx].cost;

				idx = pIdx->Idx;

				pIdx = pIdx->pNext;

				while (pIdx)
				{
					cost = InArray.Element[pIdx->Idx].cost;

					if (cost < min)
					{
						min = cost;

						idx = pIdx->Idx;
					}

					pIdx = pIdx->pNext;
				}

				OutArray.Element[OutArray.n++] = InArray.Element[idx];

				break;
			}

			pIdx = binArray[iTgt].mem;

			for (i = 0; i < nBins_; i++)
			{
				bin = binArray[iTgt].listArray.Element + i;

				RVLQLIST_INIT(bin);

				n[iTgt][i] = 0;
			}

			pIdx = binArray[iSrc].listArray.Element[iSrcBin].pFirst;

			min = max = InArray.Element[pIdx->Idx].cost;

			CostType cost;

			pIdx = pIdx->pNext;

			while (pIdx)
			{
				cost = InArray.Element[pIdx->Idx].cost;

				if (cost < min)
					min = cost;
				else if (cost > max)
					max = cost;

				pIdx = pIdx->pNext;
			}

			CostType binSize = 1.01 * (max - min) / (CostType)nBins_;

			if (binSize == 0.0)
			{
				pIdx = binArray[iSrc].listArray.Element[iSrcBin].pFirst;

				while (pIdx && OutArray.n < nOut)
				{
					OutArray.Element[OutArray.n++] = InArray.Element[pIdx->Idx];

					pIdx = pIdx->pNext;
				}

				break;
			}

			QLIST::Index2 *pIdx_ = binArray[iTgt].mem;

			int iBin;

			pIdx = binArray[iSrc].listArray.Element[iSrcBin].pFirst;

			while (pIdx)
			{
				cost = InArray.Element[pIdx->Idx].cost;

				iBin = (int)((cost - min) / binSize);

				bin = binArray[iTgt].listArray.Element + iBin;

				RVLQLIST_ADD_ENTRY(bin, pIdx_);

				pIdx_->Idx = pIdx->Idx;

				pIdx_++;

				n[iTgt][iBin]++;

				pIdx = pIdx->pNext;
			}

			iBin = 0;

			while (OutArray.n < nOut)
			{
				bin = binArray[iTgt].listArray.Element + iBin;

				if (OutArray.n + n[iTgt][iBin] <= nOut)
				{
					pIdx = bin->pFirst;

					while (pIdx)
					{
						OutArray.Element[OutArray.n++] = InArray.Element[pIdx->Idx];

						pIdx = pIdx->pNext;
					}

					iBin++;
				}
				else
				{
					iSrcBin = iBin;

					iTmp = iSrc;
					iSrc = iTgt;
					iTgt = iTmp;

					nBins_ = n[iSrc][iSrcBin] / (nOut - OutArray.n) + 1;

					if (nBins_ > nBins)
						nBins_ = nBins;

					break;
				}
			}
		}

		for (i = 0; i < 2; i++)
		{
			delete[] binArray[i].listArray.Element;
			delete[] binArray[i].mem;
			delete[] n[i];
		}
	}

#ifdef RVLVTK
	class RVLVTKPLYWriter : public vtkPLYWriter
	{
	public:
		static RVLVTKPLYWriter *New();
		void WriteDataWithNormals();
		void WritePolyData(
			std::string fileName,
			vtkSmartPointer<vtkPolyData> pPolygonData);
	};

	vtkSmartPointer<vtkPolyData> DisplayIsoSurface(
		Array3D<float> f,
		float *P0,
		float voxelSize,
		float isolevel,
		float *W = NULL);

#endif

	// dM = dX' * inv(C) * dX

	template <typename T>
	T MahalanobisDistance(CvMat *dX, CvMat *C, CvMat *Tmp)
	{
		if (cvSolve(C, dX, Tmp))
			return cvDotProduct(dX, Tmp);
		else
			return -1.0;
	}

	// Computes the roots of a cubic polynomial. Polinomial coefficients are given in array p and the results are stored in array z.

	template <typename T>
	void Roots3(T *p, T *z, bool *bReal)
	{
		T q0 = p[0] / p[3];
		T q1 = p[1] / p[3];
		T q2 = p[2] / p[3];
		T q2_2 = q2 * q2;
		T r = q1 / 3.0 - q2_2 / 9.0;
		T s = (q2_2 / 27.0 - q1 / 6.0) * q2 + 0.5 * q0;
		T D = s * s + r * r * r;
		T t, u, v;
		T fTmp;

		if (D > 1e-20)
		{
			t = sqrt(D);
			fTmp = -s + t;
			u = pow(fabs(fTmp), 1.0 / 3.0);
			if (fTmp < 0)
				u = -u;
			fTmp = -s - t;
			v = pow(fabs(fTmp), 1.0 / 3.0);
			if (fTmp < 0)
				v = -v;
			z[0] = u + v - q2 / 3.0;
			bReal[0] = true;
			bReal[1] = bReal[2] = false;
		}
		else if (D < -1e-20)
		{
			T w = sqrt(fabs(r));
			T cp = s / (w * w * w);
			// z[0] = - 2.0 * w * cp - q2 / 3.0;
			// T phi = acos(w);
			T phi = acos(cp);
			z[0] = -2.0 * w * cos(phi / 3.0) - q2 / 3.0;
			/////
			z[1] = 2.0 * w * cos((PI - phi) / 3.0) - q2 / 3.0;
			z[2] = 2.0 * w * cos((PI + phi) / 3.0) - q2 / 3.0;
			bReal[0] = bReal[1] = bReal[2] = true;
		}
		else
		{
			z[0] = z[1] = z[2] = -q2 / 3.0;

			bReal[0] = bReal[1] = bReal[2] = false;
		}
	}

	// Computes eigenvalues of a 3x3 symmetric matrix C.

	template <typename T>
	void EigCov3(T *C, T *eig, bool *bReal)
	{
		T p[4];

		T c0101 = C[0 * 3 + 1] * C[0 * 3 + 1];
		T c0202 = C[0 * 3 + 2] * C[0 * 3 + 2];
		T c1212 = C[1 * 3 + 2] * C[1 * 3 + 2];

		p[3] = 1.0;
		p[2] = -C[0 * 3 + 0] - C[2 * 3 + 2] - C[1 * 3 + 1];
		p[1] = -c0202 + C[0 * 3 + 0] * C[2 * 3 + 2] + C[1 * 3 + 1] * C[2 * 3 + 2] - c0101 + C[0 * 3 + 0] * C[1 * 3 + 1] - c1212;
		p[0] = -C[0 * 3 + 2] * C[0 * 3 + 1] * C[1 * 3 + 2] + C[0 * 3 + 0] * c1212 +
			   c0202 * C[1 * 3 + 1] + c0101 * C[2 * 3 + 2] -
			   C[0 * 3 + 0] * C[1 * 3 + 1] * C[2 * 3 + 2] - C[0 * 3 + 1] * C[0 * 3 + 2] * C[1 * 3 + 2];

		Roots3<T>(p, eig, bReal);
	}

	// This function is an implementation of the algorithm proposed in ritter_90.

	template <typename T>
	void BoundingSphere(
		Array2D<T> points,
		T *center,
		T &radius)
	{
		Box<T> box;

		int imin[3], imax[3];

		RVLNULL3VECTOR(imin);
		RVLNULL3VECTOR(imax);

		int i, j;
		T *P;
		T min_, max_;

		for (i = 1; i < points.h; i++)
		{
			P = points.Element + 3 * i;

			for (j = 0; j < 3; j++)
			{
				min_ = points.Element[3 * imin[j] + j];

				if (P[j] < min_)
				{
					imin[j] = i;

					min_ = P[j];
				}

				max_ = points.Element[3 * imax[j] + j];

				if (P[j] > max_)
				{
					imax[j] = i;

					max_ = P[j];
				}
			}
		}

		T maxd2 = 0.0;

		int imaxd;
		T dP[3];
		T *P_;
		T d2;

		for (j = 0; j < 3; j++)
		{
			P = points.Element + 3 * imin[j];
			P_ = points.Element + 3 * imax[j];

			RVLDIF3VECTORS(P_, P, dP);

			d2 = RVLDOTPRODUCT3(dP, dP);

			if (d2 > maxd2)
			{
				maxd2 = d2;

				imaxd = j;
			}
		}

		T radius2 = 0.25 * maxd2;
		radius = sqrt(radius2);

		P = points.Element + 3 * imin[imaxd];
		P_ = points.Element + 3 * imax[imaxd];

		RVLSUM3VECTORS(P, P_, center);
		RVLSCALE3VECTOR(center, 0.5, center);

		T dist, dist2, s;
		T V3Tmp[3];

		for (i = 0; i < points.h; i++)
		{
			P = points.Element + 3 * i;

			RVLDIF3VECTORS(P, center, dP);

			dist2 = RVLDOTPRODUCT3(dP, dP);

			if (dist2 <= radius2)
				continue;

			dist = sqrt(dist2);

			radius = 0.5 * (radius + dist);

			radius2 = radius * radius;

			s = dist - radius;

			RVLSCALE3VECTOR(center, radius, center);
			RVLSCALE3VECTOR(P, s, V3Tmp);
			RVLSUM3VECTORS(center, V3Tmp, center);
			RVLSCALE3VECTOR2(center, dist, center);
		}
	}

	// Function RVLGaussRandBM() is implemented according to the code given at
	// http://en.wikipedia.org/wiki/Box-Muller_transform

	template <typename T>
	T GaussRandBM(T std)
	{
		T z;

		do
		{
			z = (T)rand() / (T)RAND_MAX;
		} while (z < 1e-10);

		return std * sqrt(-2.0 * log(z)) * cos(2.0 * PI * ((double)rand() / (double)RAND_MAX));
	}

	template <typename T>
	T GaussPseudoRandBM(
		Array<int> rndVal,
		int &iRndVal)
	{
		int z;
		do
			RVLRND(1000000, rndVal.Element, rndVal.n, iRndVal, z)
		while (z == 0);
		T u1 = 1e-6 * (T)z;
		RVLRND(1000000, rndVal.Element, rndVal.n, iRndVal, z);
		T u2 = 1e-6 * (T)z;

		return sqrt(-2.0 * log(u1)) * cos(2.0 * PI * u2);
	}

	template <typename T>
	T RealPseudoRand(
		Array<int> rndVal,
		int &iRndVal)
	{
		int z;
		RVLRND(1000000, rndVal.Element, rndVal.n, iRndVal, z);
		return 1e-6 * (T)z;
	}

#ifdef RVLHDF5
	template <typename T>
	void ReadFromHDF5(
		std::string fileName,
		std::string dataName,
		int nDims,
		H5::DataType dataType,
		T *&data,
		hsize_t *dims)
	{
		H5::H5File depthImageFile(std::string(fileName), H5F_ACC_RDONLY);
		H5::DataSet depthImageTensor = depthImageFile.openDataSet(dataName);

		H5::DataSpace fileSpace = depthImageTensor.getSpace();

		fileSpace.getSimpleExtentDims(dims);

		data = new T[dims[0] * dims[1] * dims[2]];
		depthImageTensor.read(data, dataType);
	}

	template <typename T>
	void WriteToHDF5(
		std::string fileName,
		std::string dataName,
		int nDims,
		hsize_t *dims,
		H5::DataType dataType,
		T *data)
	{
		H5::H5File featureTensorFile(fileName, H5F_ACC_TRUNC);

		float fillValue = 0.0f;
		H5::DSetCreatPropList propList;
		propList.setFillValue(H5::PredType::NATIVE_FLOAT, &fillValue);

		H5::DataSpace dataSpace(nDims, dims);

		H5::DataSet dataset = featureTensorFile.createDataSet(dataName, dataType, dataSpace, propList);

		dataset.write(data, dataType);
	}
#endif

	void CreateBoxMesh(
		float *size,
		float *P,
		int *faces,
		int faceIdxOffset = 0);
	bool LineRectIntersection(
		float *lineP1,
		float *lineP2,
		Rect<float> rect,
		float *intersectionP1,
		float *intersectionP2);
	bool LineConvexSetIntersection2D(
		float *line,
		Array<Vector3<float>> convexSet,
		float *intersectionLineSegment);

	// Vidovic
	void UnionOfIndices(
		Array<int> &iInArray1,
		Array<int> &iInArray2,
		Array<int> &iOutArray,
		bool *bIndicesInArray);

	bool CheckFlag(unsigned char flags, unsigned char flagToCheck);
	bool CheckFlags(unsigned char flags, unsigned char flagsToCheck);

	class Queue
	{
	public:
		Queue(int maxKeyIn);
		~Queue();
		void Add(
			int key,
			QLIST::Index *pIdx);
		int FetchAndRemove(int key);
		QLIST::Index **GetPtrToLast();
		void Remove(
			int key,
			QLIST::Index **ppIdx);
		void Move(
			int key,
			QLIST::Index **ppIdx,
			int newKey);

	public:
		int nKeys;
		QList<QLIST::Index> *list;
		int maxKey;
	};

	bool IdxCostPairComparison(Pair<int, float> x, Pair<int, float> y);
	bool IdxCostPairComparisonDesc(Pair<int, float> x, Pair<int, float> y);

	// This class is designed for storing data into files.
	// The format of the file created and read by DataStorage consists of an ascii header followed by binary data.

#define RVLDATASTRUCTURE_TYPE_NONE 0
#define RVLDATASTRUCTURE_TYPE_BOOL 1
#define RVLDATASTRUCTURE_TYPE_INT 2
#define RVLDATASTRUCTURE_TYPE_FLOAT 3
#define RVLDATASTRUCTURE_TYPE_DOUBLE 4
#define RVLDATASTRUCTURE_TYPE_ARRAY 5
#define RVLDATASTRUCTURE_TYPE_STRUCT 6

#define RVLDATASTORAGE_MAX_ARRAY_DIMS 10
#define RVLDATASTORAGE_MAX_NAME_LENGTH 200

	struct DataStructureElement
	{
		std::string name;
		uchar type;
		Array<int> idx;
		int iParent;
		std::vector<int> children;
	};

	class DataStorage
	{
	public:
		DataStorage();
		virtual ~DataStorage();
		void Clear();
		bool Load(std::string fileName);

	private:
		std::string LoadName();
		void LoadStructureElement(std::string name = "");
		void LoadValue(int iData);
		void LoadArray(int iData);
		void LoadStruct(int iData);
		void AddChild(
			int iParent,
			int iChild);

	private:
		FILE *fp;
		std::vector<DataStructureElement> structure;
		Array<int> idx;
		int iParent;
		char c;
	};
} // namespace RVL
