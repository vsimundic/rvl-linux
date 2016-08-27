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
}

