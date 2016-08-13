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
	void QuickSort(int *Key, int *Index, int n);
}
