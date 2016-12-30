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

namespace RVL
{
	template <typename T> struct SortIndex
	{
		int idx;
		T cost;
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
	};

	bool GetAngleAxis(float *R, float *V, float &theta);
	void GetDistance(float *t, float &distance);
	//END VIDOVIC

	void QuickSort(int *Key, int *Index, int n);
	void RandomColor(unsigned char *color);

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
}

