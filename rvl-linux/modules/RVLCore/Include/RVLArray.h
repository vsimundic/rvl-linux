template <class Type> struct RVLARRAY_
{
	Type *Element;
	int n;
};

struct RVLARRAY
{
	BYTE *pFirst;
	BYTE *pEnd;
};

struct RVLARRAY2
{
	BYTE *pFirst;
	BYTE *pLast;
	BYTE *pData;
};

struct RVL2DARRAY
{
	BYTE *m_vp;
	int m_ElementSize;
	int m_Width;
	int m_Height;
};

namespace RVL
{
	template <typename Type> struct Array
	{
		Type *Element;
		int n;
	};

	template <typename Type> struct Array2D
	{
		Type *Element;
		int w;
		int h;
	};

	template <typename Type> struct Array3D
	{
		Type *Element;
		int a;
		int b;
		int c;
	};
}

#define RVLARRAY_RESIZE(Array, Type, size) {Array.n = size; if(size > Array.n){RVL_DELETE_ARRAY(Array.Element); Array.Element = new Type[size];}}
#define RVL3DARRAY_ELEMENT_INDEX(Array, x, y, z)	(Array.a * (Array.b * (z) + (y)) + (x))
#define RVL3DARRAY_ELEMENT(Array, x, y, z)	(Array.Element + RVL3DARRAY_ELEMENT_INDEX(Array, x, y, z))