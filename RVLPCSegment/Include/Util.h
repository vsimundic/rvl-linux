#define RVL_DELETE_ARRAY(Array) {if(Array)delete[] Array; Array = NULL;}
#define RVLSCALECOLOR(SrcColor, a, TgtColor)\
{\
	TgtColor[0] = (unsigned char)((int)(SrcColor[0]) * a / 100);\
	TgtColor[1] = (unsigned char)((int)(SrcColor[1]) * a / 100);\
	TgtColor[2] = (unsigned char)((int)(SrcColor[2]) * a / 100);\
}
#define RVLSCALECOLOR2(SrcColor, scale, TgtColor)\
{\
	TgtColor[0] = (unsigned char)((int)(SrcColor[0]) * scale[0] / 100);\
	TgtColor[1] = (unsigned char)((int)(SrcColor[1]) * scale[1] / 100);\
	TgtColor[2] = (unsigned char)((int)(SrcColor[2]) * scale[2] / 100);\
}
#define RVLGETFILEEXTENSION(FileName)	(strrchr(FileName, '.') + 1)
#define RVLCROPRECT(minx, maxx, miny, maxy, left, right, top, bottom)\
{\
	if(top < miny)\
		top = miny;\
	if (bottom > maxy)\
		bottom = maxy;\
	if (left < minx)\
		left = minx;\
	if(right > maxx)\
		right = maxx;\
}

namespace RVL
{
	template <typename T> struct SortIndex
	{
		int idx;
		T cost;
	};

	template <typename T1, typename T2> struct Pair
	{
		T1 a;
		T2 b;
	};

	//VIDOVIC
	struct GTInstance{
		int iScene;
		int iModel;
		float R[9];
		float t[3];
		bool matched;
	};

	struct SegmentGTInstance{
		int iScene;
		int iSSegment;
		int iModel;
		int iMSegment;
		bool valid;
	};

	bool GetAngleAxis(float *R, float *V, float &theta);
	void GetDistance(float *t, float &distance);
	//END VIDOVIC
	void PrintMatrix(FILE *fp, double *A, int n, int m);

	void QuickSort(int *Key, int *Index, int n);
	void RandomColor(unsigned char *color);
	void RandomColors(
		unsigned char *SelectionColor,
		unsigned char *&colorArray,
		int n);

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

	void GetFileNameAndPath(
		char *fileNameWithPath,
		char *&fileName,
		char *&filePath);

	//VIDOVIC
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

		bool Init(char *filePath, char *GTFolderPath, char *modelsID);
		bool Init(FileSequenceLoader sceneSequence, char *GTFolderPath, char *modelsID);
		bool SaveGTFile(char *filePath);
		void ResetMatchFlag();

	private:
		bool LoadModels(char *filePath);
		int FindModelID(char *modelName);
		void CreateGTFilePath(char *scenePath, char *GTFilePath);

	public:
		int nScenes;
		int nModels;
		Array<Array<GTInstance>> GT;

	private:
		int iScene;
		char *modelsInDB;
		char *GTFolder;
		char *GTFilePath;

	};
	//END VIDOVIC

	template<typename T> struct QList2Array		// Move to RVLQList.h
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

	vtkSmartPointer<vtkPolyData>  DisplayIsoSurface(
		Array3D<float> f,
		float *P0,
		float voxelSize,
		float isolevel);
}	// namespace RVL

