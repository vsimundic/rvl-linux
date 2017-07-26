#define APPROX_ZERO		1e-10
#define DOUBLE2INT(x)	(int)floor(x + 0.5)
#define DOUBLE2BYTE(x)	(BYTE)floor(x + 0.5)
#define RVLGETFILEEXTENSION(FileName)	(strrchr(FileName, '.') + 1)

struct PIX_ARRAY
{
	int Width;
	int Height;
	int nPixBytes;
	unsigned char *pPix;
	BOOL bOwnData;
	BOOL bColor;
};

struct RVL2DMOMENTS
{
	int n;
	double S[2], S2[4];
};

struct RVL3DMOMENTS
{
	int n;
	double S[3], S2[9];
};

struct RVL2DOBB
{
	double u0, v0, cs, sn, w, h;
};

struct RVLHASHTABLE_ENTRY
{
	DWORD Data;
	RVLHASHTABLE_ENTRY *pNextEntry;
};

struct RVLRANDPDFLT
{
	int min;
	int max;
	int n;
	double nrm;
	double dval;
	double *LT;
};

struct RVLPDF_BILATERAL_GAUSSIAN
{
	double mean;
	double sigN;
	double sigP;
	double k;
};

int Max(int *x, int n);
int Min(int *x, int n);
double Max(double *x, int n);
double Min(double *x, int n);
int Max(int *x, int n, int &j);
int Min(int *x, int n, int &j);
double Max(double *x, int n, int &j);
float Max(float *x, int n, int &j);
double Min(double *x, int n, int &j);
float Min(float *x, int n, int &j);
BOOL TwoEqsTwoVars(double a11, double a12, double a21, double a22, double b1, double b2, double &x1, double &x2);
BOOL TwoEqsTwoVars(int a11, int a12, int a21, int a22, int b1, int b2, int &x1, int &x2);
BOOL TwoEqsTwoVars(int a11, int a12, int a21, int a22, int b1, int b2, double &x1, double &x2);
void GaussJordan3(double *A, double *B, double *X);
void MatrixMultiplication(double *A, double *B, double *Z, int n, int m);
void MatrixMultiplication(double *A, double *B, double *Z, int n1, int n2, int n3);
void MatrixMultiplicationT(double *A, double *B, double *Z, int n, int m);
void MatrixMultiplicationT(double *A, double *B, double *Z, int n1, int n2, int n3);
void MatrixMultiplicationT2(double *A, double *B, double *Z, int n1, int n2, int n3);
void rtUtilInit();
int rtsqrt(int x);
BYTE rtAngle90(int dx, int dy);
BYTE rtAngle180(int dx, int dy);
BYTE rtAngle180_2(int dx, int dy);
BYTE rtAngle360(int dx, int dy);
BYTE rtlog10(int x);
BYTE *CreateLogLookUpTable(int MinX, int MaxX, int CellSize, double Base);
void PanTiltRoll(double PanAngle, double TiltAngle, double RollAngle, double *Rotation/*, double *InvRotation*/);
void PanTiltRoll(double *Rot, double &PanAngle, double &TiltAngle, double &RollAngle);
void PTR2RPT(double PanPTR, double TiltPTR, double RollPTR,
	     double &PanRPT, double &TiltRPT, double &RollRPT);
void RPT2PTR(double PanRPT, double TiltRPT, double RollRPT,
	     double &PanPTR, double &TiltPTR, double &RollPTR);
void InverseRotation(double *InvRotation, double *Rotation);
void InverseTransform3D(double *InvRotation, double *InvTranslation, double *Rotation,
						double *Translation);
void RotFrom3DPoints(double *Rot, int *P11, int *P12, int *P21, int *P22);
void PTRFrom3DPoints(double &Pan, double &Tilt, double &Roll, double &r22, double &r33, int *P11, int *P12, int *P21, int *P22);
void RVLCombineTransform3D(double *Rot1, double *Trans1, double *Rot2, double *Trans2,
						double *Rot, double *Trans);
void InverseMatrix3(double *InvA, double *A);
BOOL InverseMatrix3(double *InvA, double *A, double MinDet);
BOOL InverseMatrix2(double *InvA, double *A, double MinDet);
void Sort(int *Key, int *Index, int n, BOOL bIndex = FALSE);
void Sort(double *Key, int *Index, int n, BOOL bIndex = FALSE);
void ANDDWArray(DWORD *X1, DWORD *X2, DWORD *Y, int n);
void ORDWArray(DWORD *X1, DWORD *X2, DWORD *Y, int n);
BOOL EqualDWArray(DWORD *X1, DWORD *X2, int n);
BOOL IsInAngle(int Angle, int AngleMin, int AngleMax);
void CrossProduct(double *X1, double *X2, double *Y);
void MultiplicationWithTransposed(double *A, double *Z, int n, int m);
BOOL GetMinEigenvector(double *A, double *q, double &d, int n);
void Robot2Camera(double *RotR, double *RotC);
BOOL Roots2(double *p, double *z);
void Roots3(double *p, double *z, BOOL *bReal);
void Roots4(double *p, double *z, BOOL *bReal);
void LinearTransform2D(double cs, double sn, double x0, double y0, double x, double y, 
					   double &p, double &q);
void InvLinearTransform3D(double *A, double *b, double *X, double *Y);
void LinearTransform2D(int dx, int dy, int len, int x0, int y0, int x, int y, int &p, int &q);
int Point2Line2DDist(int dx, int dy, int len, int x0, int y0, int x, int y);
void LinearTransform2D2(double cs, double sn, double x0, double y0, double x, double y, 
					   double &p, double &q);
void Get2DRot(double *Rot, double Alpha);
void Rotxz(double cs, double sn, double *X, double *Y);
void GetEllipseParams(double c11, double c12, double c22, double &R1, double &R2, 
					  double &tgPhi);
BOOL GetEllipseAxis(double c11, double c12, double c22, double &p1, double &p2, double &tgPhi);
void RVLQuickSort(short *Key, int *Index, int n);
//template <class Type>							// for Nyarko
//void RVLBubbleSort(CRVLMPtrChain *pInList,		// for Nyarko
//				   Type **OutArray,				// for Nyarko		
//				   BOOL descending = FALSE);	// for Nyarko
BOOL RVLEig2(double *C, double *eig);
BOOL RVLGetMaxEigVector2(double *C, double *eig, double *Veig);
void RVLEig3(double *C, double *eig, BOOL *bReal);
BOOL RVLGetMinEigVector3(double *C, double *eig, BOOL *bReal, double *Veig);
BOOL RVLGetMaxEigVector3(double *C, double *eig, BOOL *bReal, double *Veig);
void RVLGetCovMatrix2(RVL2DMOMENTS *pMoments, double *C, double *M);
void RVLGetCovMatrix3(RVL3DMOMENTS *pMoments, double *C, double *M);
void CreateUnitMatrix(double *A, int n);
void Sort3(double *X);
void RotCov2D(double *C, double cs, double sn, double *CRot);
void RotCov3D(double *C, double *Rot, double *CRot);
void RVLGetMainAxisOfCov3D(double *C, double *W3D);
BOOL RVLGetAxesOfCov3D(double *C, double *eig, double *VNrm, double *V, double *lV);
void RVLUnitMatrix(double *M, int n);
double RVLMahalanobisDistance2D(double *dX, double *C2D);
double RVLMahalanobisDistance3D(double *dX, double *C3D);
double RVLMahalanobisDistance(CvMat *dX, CvMat *C, CvMat *Tmp);
void RVLHSB2RGB(int h, int s, int k, int &r, int &g, int &b);
int *RVLGetsqrt_LookUpTable();
void RVLMomentsSum(RVL3DMOMENTS *pMoments1, RVL3DMOMENTS *pMoments2, RVL3DMOMENTS *pMomentsTgt);
BOOL RVLInsideTriangle(double Ax, double Ay, double Bx, double By, double Cx, double Cy, 
					   double Px, double Py);
void RVLGetCov3DFromCov2D(double *C2D, double *J, double *C3D);
void RVLGetCov2DFromCov3D(double *C3D, double *J, double *C2D);
double RVLGetCov1DFromCov3D(double *C3D, double *J);
double RVLGetCov1DFromCov2D(double *C2D, double *J);
void RVLCovTransf(double *CIn, double *J, int n, int m, double *COut);
void RVLVector2Matrix(double *X, int n, double *Y);
void RVLCompleteCov(double *C, int n);
void RVLCovTransf(CvMat *m_Csrc, 
				  CvMat *m_J,
				  CvMat *m_Ctgt,
				  CvMat *m_Tmp);
BOOL RVLGetEigVects3(double *C, 
				     double *eig, 
				     double *V);
void RVLPrintMatrix(FILE *fp, double *A, int n, int m);
double RVLGaussRand(double std);
double RVLGaussRandBM(double std);
void RVLResize(PIX_ARRAY *pPixArray, 
			   int Size);
void RVLRandomSelect(void **SrcArray, 
					 int n,
					 int m,
					 void **TgtArray);
void **RVLCreatePtrArray(CRVLMPtrChain *pSrcArray,
		  	             CRVLMem *pMem);
void RVLPrintCov(FILE *fp,
				 CvMat *C,
				 CvMat *W,
				 CvMat *U,
				 double k = 1.0);
void RVLPrintCov(FILE *fp,
				 double *C,
				 int n,
				 double k = 1.0);
void RVLInitMatrices();
void **RVLChainToArray(CRVLMPtrChain *pChain, 
					   CRVLMem *pMem,
					   int &n);
int RVLSqrDistance2LineSeg(	int *iU, 
							int *iU1, int *iU2);
void RVLGrayscaleToRGB(unsigned char *pPixSrc,
					   int w, int h,
					   unsigned char *pPixTgt);
void RVLCreateEmptyRegionGrowingMap(unsigned short *RegionGrowingMap,
									int w, int h,
									BYTE Empty,
									BYTE Border);
char *RVLCreateString(char *strIn);
void RVLCopyString(char *strSrc, char **pstrTgt, CRVLMem *pMem = NULL);
char *RVLCreateFileName(char *SrcFileName, 
						char *SrcExtension,
						int n,
						char *TgtExtension,
						CRVLMem *pMem = NULL);
void RVLCopyPixArray(PIX_ARRAY *SrcImage,
					 PIX_ARRAY *TgtImage);
void RVLCopyImageToPixArray(IplImage *pSrcImage,
							PIX_ARRAY *pTgtImage);
void Skew(double *V, double *M);
int* RVLWhereAbove(int *x, int n, double l, int &j);
int* RVLWhereAbove(float *x, int n, double l, int &j);
int* RVLWhereAbove(double *x, int n, double l, int &j);
void RVLSetFileNumber(char *FileName, char *Extension, int n);
int RVLGetFileNumber(char *FileName, char *Extension);
BOOL RVLGetNextFileName(char *FileName, char *Extension, int maxiSample, int diSample = 1);
BOOL RVLGetFirstValidFileName(char *FileName, char *Extension, int maxiSample, int iSample_ = -1);
void RVL3x3x3BlockMxTo6x6(double *PSrc, double *PTgt);
void RVL6DOFCovTransf(double *C,
					  double *Jqq,
					  double *Jqt,
					  double *Jtt,
					  double *COut);
void RVL3DOFInvTransfUncert(double csalphaAB, 
							double snalphaAB,
							double txBA,
							double tyBA,
							double *C,
							double *invC);
void RVL6DOFInvTransfUncert(double *R,
							double *t,
							double *C,
							double *invC);
void RVLZoom(IplImage *pSrcImage, IplImage *pTgtImage, int ZoomFactor);
double RVLTriangularPDF(double val,
						double Val0,
						double mindVal,
						double maxdVal);
double RVLBilateralGaussianPDF(	double val,
								double Val0,
								double mindVal,
								double maxdVal);
double RVLBilateralGaussianPDFAngle(double val,
									double Val0,
									double mindVal,
									double maxdVal);
double RVLLogBilateralGaussianPDF(	double val,
									double Val0,
									double mindVal,
									double maxdVal);
double RVLLogBilateralGaussianPDFAngle(double val,
									double Val0,
									double mindVal,
									double maxdVal);
double RVLBilateralGaussianPDF(	double val,
								void *vpParams);
void RVLRandPDFLT(	double PDF(double val, void *vpParams),
					void *vpParams_,
					double min,
					double max,
					double dval,
					double nrm,
					RVLRANDPDFLT *pLT);
double RVLRandPDF(RVLRANDPDFLT *pLT);
double RVLRandPDF(	double PDF(double val_, void *vpParams_),
					void *vpParams,
					double min,
					double max,
					double dval);

inline int rthypot(int x, int y)
{
	return rtsqrt(x * x + y * y);
}

inline int RVLRandom(int yMin, int yMax)
{
	return yMin + (yMax - yMin) * rand() / RAND_MAX;
}

inline double RVLRandom(double yMin, double yMax)
{
	return yMin + (yMax - yMin) * ((double)rand() / (double)RAND_MAX);
}

// Y(3 x 1) = A(3 x 3) * X(3 x 1)

inline void LinearTransform3D(double *A, double *X, double *Y)
{
	Y[0] = A[0 + 0 * 3] * X[0] + A[1 + 0 * 3] * X[1] + A[2 + 0 * 3] * X[2];
	Y[1] = A[0 + 1 * 3] * X[0] + A[1 + 1 * 3] * X[1] + A[2 + 1 * 3] * X[2];
	Y[2] = A[0 + 2 * 3] * X[0] + A[1 + 2 * 3] * X[1] + A[2 + 2 * 3] * X[2];
}

// Y(3 x 1) = A'(3 x 3) * X(3 x 1)

inline void LinearTransform3DT(double *A, double *X, double *Y)
{
	Y[0] = A[0 + 0 * 3] * X[0] + A[0 + 1 * 3] * X[1] + A[0 + 2 * 3] * X[2];
	Y[1] = A[1 + 0 * 3] * X[0] + A[1 + 1 * 3] * X[1] + A[1 + 2 * 3] * X[2];
	Y[2] = A[2 + 0 * 3] * X[0] + A[2 + 1 * 3] * X[1] + A[2 + 2 * 3] * X[2];
}

// Y(3 x 1) = A(3 x 3) * X(3 x 1) + b(3 x 1)

inline void LinearTransform3D(double *A, double *b, double *X, double *Y)
{
	Y[0] = A[0 + 0 * 3] * X[0] + A[1 + 0 * 3] * X[1] + A[2 + 0 * 3] * X[2] + b[0];
	Y[1] = A[0 + 1 * 3] * X[0] + A[1 + 1 * 3] * X[1] + A[2 + 1 * 3] * X[2] + b[1];
	Y[2] = A[0 + 2 * 3] * X[0] + A[1 + 2 * 3] * X[1] + A[2 + 2 * 3] * X[2] + b[2];
}

inline double RVLDotProduct(double *X1, double *X2)
{
	return X1[0] * X2[0] + X1[1] * X2[1] + X1[2] * X2[2];
}

inline double RVLGet3DVectorLen(double *V3D)
{
	return sqrt(V3D[0] * V3D[0] + V3D[1] * V3D[1] + V3D[2] * V3D[2]);
}

inline void RVLNormalize3DVector(double *V3D)
{
	double l = sqrt(V3D[0] * V3D[0] + V3D[1] * V3D[1] + V3D[2] * V3D[2]);

	V3D[0] /= l;
	V3D[1] /= l;
	V3D[2] /= l;
}

inline void RVLSum4D(double *X1, double *X2, double *Y)
{
	Y[0] = X1[0] + X2[0];
	Y[1] = X1[1] + X2[1];
	Y[2] = X1[2] + X2[2];
	Y[3] = X1[3] + X2[3];
}

inline void RVLSum3D(double *X1, double *X2, double *Y)
{
	Y[0] = X1[0] + X2[0];
	Y[1] = X1[1] + X2[1];
	Y[2] = X1[2] + X2[2];
}

inline void RVLSum3DWeighted(double *X1, double *X2, double w, double *Y)
{
	Y[0] = X1[0] + w * X2[0];
	Y[1] = X1[1] + w * X2[1];
	Y[2] = X1[2] + w * X2[2];
}

inline void RVLDif4D(double *X1, double *X2, double *Y)
{
	Y[0] = X1[0] - X2[0];
	Y[1] = X1[1] - X2[1];
	Y[2] = X1[2] - X2[2];
	Y[3] = X1[3] - X2[3];
}

inline void RVLDif3D(double *X1, double *X2, double *Y)
{
	Y[0] = X1[0] - X2[0];
	Y[1] = X1[1] - X2[1];
	Y[2] = X1[2] - X2[2];
}

inline void RVLDif2D(double *X1, double *X2, double *Y)
{
	Y[0] = X1[0] - X2[0];
	Y[1] = X1[1] - X2[1];
}

inline void RVLBoundingBoxUpdate(double p, double q, double &minp, double &maxp, 
								 double &minq, double &maxq, BOOL &bFirst)
{
	if(bFirst)
	{
		bFirst = FALSE;

		minp = maxp = p;
		minq = maxq = q;
	}
	else
	{
		if(p < minp)
			minp = p;
		else if(p > maxp)
			maxp = p;

		if(q < minq)
			minq = q;
		else if(q > maxq)
			maxq = q;	
	}
};

inline void RVLBoundingBoxUpdate(int p, int q, int &minp, int &maxp, 
								 int &minq, int &maxq, BOOL &bFirst)
{
	if(bFirst)
	{
		bFirst = FALSE;

		minp = maxp = p;
		minq = maxq = q;
	}
	else
	{
		if(p < minp)
			minp = p;
		else if(p > maxp)
			maxp = p;

		if(q < minq)
			minq = q;
		else if(q > maxq)
			maxq = q;	
	}
};


inline void RVLAngleDiff(double phi1, double phi2, double &dphi)
{
	dphi = phi2 - phi1;

	if(dphi > PI)
		dphi -= 2.0 * PI;
	else if(dphi < -PI)
		dphi += 2.0 * PI;
};

// M(3x3) = X1(3x1) * X2(3x1)'

inline void RVLM3D(double *X1, double *X2, double *M)
{
	M[0*3+0] = X1[0] * X2[0];
	M[0*3+1] = X1[0] * X2[1];
	M[0*3+2] = X1[0] * X2[2];

	M[1*3+0] = X1[1] * X2[0];
	M[1*3+1] = X1[1] * X2[1];
	M[1*3+2] = X1[1] * X2[2];

	M[2*3+0] = X1[2] * X2[0];
	M[2*3+1] = X1[2] * X2[1];
	M[2*3+2] = X1[2] * X2[2];
}

// M(3x3) = M(3x3) + X1(3x1) * X2(3x1)'

inline void RVLUpdateM3D(double *X1, double *X2, double w, double *M)
{
	M[0*3+0] += (w * X1[0] * X2[0]);
	M[0*3+1] += (w * X1[0] * X2[1]);
	M[0*3+2] += (w * X1[0] * X2[2]);

	M[1*3+0] += (w * X1[1] * X2[0]);
	M[1*3+1] += (w * X1[1] * X2[1]);
	M[1*3+2] += (w * X1[1] * X2[2]);

	M[2*3+0] += (w * X1[2] * X2[0]);
	M[2*3+1] += (w * X1[2] * X2[1]);
	M[2*3+2] += (w * X1[2] * X2[2]);
}

inline CvSize RVLGetPixArraySize(PIX_ARRAY *pPixArray)
{
	return cvSize(pPixArray->Width, pPixArray->Height);
}

// created by Damir Filko
// adapted for general case by Robert Cupec

template <class Type>
void RVLBubbleSort(Type **OutArray,
				   int n,
				   BOOL descending = FALSE)
{
	Type* tempVoid;
	bool chg = true;

	int i;

	while(chg)
	{
		chg = false;
		for(i = 0; i < n - 1; i++)
		{
			if (descending)
			{
				if (((Type *)OutArray[i + 1])->cost > 
					((Type *)OutArray[i])->cost)
				{
					tempVoid = OutArray[i];
					OutArray[i] = OutArray[i + 1];
					OutArray[i + 1] = tempVoid;

					chg = true;
				}
			}
			else
			{
				if (((Type *)OutArray[i + 1])->cost < 
					((Type *)OutArray[i])->cost)
				{
					tempVoid = OutArray[i];
					OutArray[i] = OutArray[i + 1];
					OutArray[i + 1] = tempVoid;

					chg = true;
				}
			}
		}
	}
}

template <class Type>
void RVLBubbleSort(CRVLMPtrChain *pInList,
				   Type **OutArray,				
				   BOOL descending = FALSE)
{
	//creating array for sorting purposes
	int n = pInList->m_nElements;
	if(OutArray == NULL)
		OutArray = new Type*[n];
	Type **ppElement = OutArray;
	Type *pElement;
	int i;

	pInList->Start();
	for(i = 0; i < n; i++)
	{
		pElement = (Type *)(pInList->GetNext());
		*(ppElement++) = pElement;
	}

	RVLBubbleSort<Type>(OutArray, n, descending);
}

// pInList is a list of RVLQLIST_PTR_ENTRY

template <class Type>
void RVLBubbleSort(RVLQLIST *pInList,
				   int n,
				   Type ***pOutArray,				
				   BOOL descending = FALSE)
{
	//creating array for sorting purposes

	Type **OutArray = *pOutArray;

	if(OutArray == NULL)
		OutArray = new Type*[n];
	Type **ppElement = OutArray;

	RVLQLIST_PTR_ENTRY *pEntry = (RVLQLIST_PTR_ENTRY *)(pInList->pFirst);

	while(pEntry)
	{
		*(ppElement++) = (Type *)(pEntry->Ptr);
		pEntry = (RVLQLIST_PTR_ENTRY *)(pEntry->pNext);
	}

	RVLBubbleSort<Type>(OutArray, n, descending);

	*pOutArray = OutArray;
}

// pInList is a list of Type structures

template <class Type>
void RVLBubbleSort2(RVLQLIST *pInList,
				   int n,
				   Type ***pOutArray,				
				   BOOL descending = FALSE)
{
	//creating array for sorting purposes

	Type **OutArray = *pOutArray;

	if(OutArray == NULL)
		OutArray = new Type*[n];
	Type **ppElement = OutArray;

	Type *pEntry = (Type *)(pInList->pFirst);

	while(pEntry)
	{
		*(ppElement++) = pEntry;
		pEntry = (Type *)(pEntry->pNext);
	}

	RVLBubbleSort<Type>(OutArray, n, descending);

	*pOutArray = OutArray;
}

template <class Type>
void RVLResetFlags(CRVLMPtrChain *pObjectList,
				   DWORD Flag)
{
	Type *pObject;

	pObjectList->Start();

	while(pObjectList->m_pNext)
	{
		pObject = (Type *)(pObjectList->GetNext());

		pObject->m_Flags &= ~Flag;
	}
}

template <class Type>
void RVLResetFlags(RVLQLIST *pObjectList,
				   DWORD Flag)
{
	Type *pObject = (Type *)(pObjectList->pFirst);

	while(pObject)
	{
		pObject->m_Flags &= ~Flag;

		pObject = (Type *)(pObject->pNext);
	}
}

// Undersampling of image Src.
// w - width of Src; h - height of Src; both w and h must be multiples of 2
// The result is stored in Tgt.
// Tgt must be allocated beforehand.

template <class Type>
void RVLUnderSampleHalf(Type *Src,
						int w, int h,
						Type *Tgt)
{
	Type *pTgt = Tgt;
	Type *pSrc = Src;
	int wHalf = w / 2;
	int hHalf = h / 2;
	Type *pTgtEnd = Tgt + wHalf * hHalf;

	Type *pTgtRowEnd;

	for(pTgtRowEnd = Tgt + wHalf; pTgtRowEnd <= pTgtEnd; pTgtRowEnd += wHalf)
	{
		for(; pTgt < pTgtRowEnd; pTgt++, pSrc += 2)
			*pTgt = *pSrc;

		pSrc += w;
	}
}

