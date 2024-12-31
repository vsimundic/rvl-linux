#pragma once

#define RVLQLIST_ASCENDING					0
#define RVLQLIST_DESCENDING					1

#define RVLQLISTARRAY_ADD_ENTRY(ListArray, i, pEntry) {*(ListArray[i].ppNext) = pEntry; ListArray[i].ppNext = &(pEntry->pNext); pEntry->pNext = NULL;}

#define RVLQLISTARRAY_ADD_ENTRY2(ListArray, i, pEntry) {pEntry->pPtrToThis = ListArray[i].ppNext; *(ListArray[i].ppNext) = pEntry; ListArray[i].ppNext = &(pEntry->pNext); pEntry->pNext = NULL;}

#define RVLQLIST_ADD_ENTRY(pList, pEntry) {*(pList->ppNext) = pEntry; pList->ppNext = &(pEntry->pNext); pEntry->pNext = NULL;}

#define RVLQLIST_ADD_ENTRY2(pList, pEntry) {pEntry->pPtrToThis = pList->ppNext; *(pList->ppNext) = pEntry; pList->ppNext = &(pEntry->pNext); pEntry->pNext = NULL;}

#define RVLQLIST_INIT(pList) {pList->pFirst = NULL; pList->ppNext = &(pList->pFirst);}

#define RVLQLIST_INIT2(pList, pList2) {pList->pFirst = NULL; pList->ppNext = &(pList2->pFirst);}

#define RVLQLIST_INSERT_ENTRY(pList, pEntry1, pEntry2, pNewEntry) {if(pList->pFirst == pEntry2){pList->pFirst = pNewEntry; pNewEntry->pNext = pEntry2;}else{pEntry1->pNext = pNewEntry; pNewEntry->pNext = pEntry2;}}

#define RVLQLIST_INSERT_ENTRY2(ppEntry, pNewEntry) {pNewEntry->pNext = (*ppEntry); *ppEntry = pNewEntry;}

#define RVLQLIST_INSERT_ENTRY2_(ppEntry, pNewEntry) {pNewEntry->pPtrToThis = ppEntry; if(*ppEntry) (*ppEntry)->pPtrToThis = &(pNewEntry->pNext); pNewEntry->pNext = (*ppEntry); *ppEntry = pNewEntry;}

#define RVLQLIST_REMOVE_ENTRY(pList, pEntry, ppEntry) {if(pEntry->pNext == NULL){pList->ppNext = ppEntry;*ppEntry = NULL;}else{*ppEntry = pEntry->pNext;}}

#define RVLQLIST_REMOVE_ENTRY2(pList, pEntry, type)\
	{if(pEntry->pNext == NULL)\
	{\
		pList->ppNext = pEntry->pPtrToThis;\
		*(pEntry->pPtrToThis) = NULL;\
	}\
	else\
	{\
		*(pEntry->pPtrToThis) = pEntry->pNext;\
		((type *)(pEntry->pNext))->pPtrToThis = pEntry->pPtrToThis;\
	}}

#define RVLQLIST_MOVE_ENTRY2(pSrcList, pTgtList, pEntry, type)\
	{\
		RVLQLIST_REMOVE_ENTRY2(pSrcList, pEntry, type);\
		RVLQLIST_ADD_ENTRY2(pTgtList, pEntry);\
	}

#define RVLQLIST_APPEND(pList, pList2)\
	{if(pList2->pFirst)\
	{\
		*(pList->ppNext) = pList2->pFirst;\
		pList->ppNext = pList2->ppNext;\
	}}

#define RVLQLIST_APPEND2(pList, pList2)\
	{if(pList2->pFirst)\
	{\
		*(pList->ppNext) = pList2->pFirst;\
		pList2->pFirst->pPtrToThis = pList->ppNext;\
		pList->ppNext = pList2->ppNext;\
	}}

#define RVLQLIST_GET_N_ENTRIES(pList, type, n) {n = 0; type *RVLQListGetnEntriesTmp = (type *)(pList->pFirst); while(RVLQListGetnEntriesTmp){n++; RVLQListGetnEntriesTmp = (type *)(RVLQListGetnEntriesTmp->pNext);}}

#define RVLQLIST_GET_N_ENTRIES2(pList, type, n, maxn) \
	{n = 0;\
	type *RVLQListGetnEntriesTmp = (type *)(pList->pFirst);\
	while(RVLQListGetnEntriesTmp)\
		{\
		n++;\
		if(n > maxn)\
			break;\
		RVLQListGetnEntriesTmp = (type *)(RVLQListGetnEntriesTmp->pNext);\
		}}

#define RVLQLIST_WRITE_TO_FILE(pList, type, fp, n, maxn) \
	{n = 0;\
	type *RVLQListWriteToFileEntry = (type *)(pList->pFirst);\
	while(RVLQListWriteToFileEntry)\
		{\
		n++;\
		if(n > maxn)\
			{fprintf(fp, "ERROR: WD STOP!"); break;}\
		fprintf(fp, "%0x ", RVLQListWriteToFileEntry);\
		RVLQListWriteToFileEntry = (type *)(RVLQListWriteToFileEntry->pNext);\
		}}

#define RVLQLISTHT_ADD_ENTRY(ListArray, pEntry, HTSize, dwTmp) \
	{dwTmp = pEntry->adr % HTSize;\
	RVLQLISTARRAY_ADD_ENTRY(ListArray, dwTmp, pEntry)\
	}

#define RVLQLISTHT_GET_ENTRY(ListArray, type, HTadr, HTSize, pEntry, pListTmp) \
	{pListTmp = ListArray + HTadr % HTSize; \
	pEntry = (type *)(pListTmp->pFirst); \
	while(pEntry) {if(pEntry->adr == HTadr) break; pEntry = (type *)(pEntry->pNext);} \
	}
#define RVLQLIST_INIT_(List) {List.pFirst = NULL; List.ppNext = &(List.pFirst);}
#define RVLQLIST_ADD_ENTRY_(List, pEntry) {*(List.ppNext) = pEntry; List.ppNext = &(pEntry->pNext); pEntry->pNext = NULL;}
#define RVLQLIST_ARRAY_INIT(Array, pQList)	{for(int i = 0; i < Array.n; i++) {pQList = Array.Element + i; RVLQLIST_INIT(pQList);}}
#define RVLQLIST_ARRAY_INIT2(pArray, Attribute, QListType)	{for(int i = 0; i < pArray->n; i++){QList<QListType> *pQList = &(pArray->Element[i].Attribute); RVLQLIST_INIT(pQList)}}
#define RVLQLIST_COPY(List, ListElementType, Field, Array, DataType)\
{\
	ListElementType *pListElement = List.pFirst;\
	DataType *pData = Array.Element;\
		while (pListElement)\
			{\
		*(pData++) = pListElement->Field;\
		pListElement = pListElement->pNext;\
			}\
	Array.n = pData - Array.Element;\
}

#define RVLQLIST_GET_NEXT_CIRCULAR(pList, pElement)	{pElement = pElement->pNext; if(!pElement) pElement = pList->pFirst;}

struct RVLQLIST	
{
	void *pFirst;
	void **ppNext;
};

struct RVLQLIST_INT_ENTRY
{
	int i;
	void *pNext;
};

struct RVLQLIST_HIST_ENTRY
{
	ushort adr;
	float value;
	void *pNext;
};

struct RVLQLIST_HIST_ENTRY_SHORT
{
	ushort adr;
	ushort value;
	void *pNext;
};

struct RVLQLIST_DISTRIBUTION_ENTRY
{
	ushort adr;
	double value;
	void *pNext;
};

struct RVLQLIST_PTR_ENTRY
{
	void *Ptr;
	void *pNext;
};

struct RVLQLIST_PTR_ENTRY2
{
	void *Ptr;
	void *pNext;
	void **pPtrToThis;
};

struct RVLQLIST_3DPT_ENTRY
{
	double X[3];
	void *pNext;
};

struct RVLQLISTHT_PTR_ENTRY
{
	DWORD adr;
	void *Ptr;
	void *pNext;
};

struct RVLQLIST_MULTILEVEL
{
	void *pFirst;
	void **ppNext;
	void *pNext;
};

//if list contains 'adr' member then adr is TRUE
template<class T>
inline RVLQLIST* InsertSortQLISTWithCopy(RVLQLIST *pList, int dir, CRVLMem *pMem, bool adr)
{
	RVLQLIST *pList_N;// = new RVLQLIST[1];
	RVLMEM_ALLOC_STRUCT(pMem, RVLQLIST, pList_N);
	RVLQLIST_INIT(pList_N);

	T *pEntry = (T *)(pList->pFirst);  
	T *pEntry_nList;
	T *pPrevEntry_nList;
	bool passed = FALSE;	//needed
	while(pEntry)
	{
		T *pNewEntry;// = new T[1];
		RVLMEM_ALLOC_STRUCT(pMem, T, pNewEntry);
		if (adr)
			pNewEntry->adr = pEntry->adr;
		pNewEntry->value = pEntry->value;
		if (!pList_N->pFirst)
		{
			RVLQLIST_ADD_ENTRY(pList_N, pNewEntry);
		}
		else
		{
			pEntry_nList = (T*)pList_N->pFirst;
			pPrevEntry_nList = (T*)pList_N->pFirst;
			passed = FALSE;
			while(pEntry_nList)
			{
				if (dir == RVLQLIST_ASCENDING)
				{
					if (pEntry_nList->value < pNewEntry->value)
					{
						if (!pEntry_nList->pNext)
						{
							RVLQLIST_ADD_ENTRY(pList_N, pNewEntry);
							break;
						}
					}
					else
					{
						if (!passed)
						{
							pList_N->pFirst = pNewEntry;
							pNewEntry->pNext = pEntry_nList;
							break;
						}
						pPrevEntry_nList->pNext = pNewEntry;
						pNewEntry->pNext = pEntry_nList;
						break;
					}
				}
				else if (dir == RVLQLIST_DESCENDING)
				{
					if (pEntry_nList->value > pNewEntry->value)
					{
						if (!pEntry_nList->pNext)
						{
							RVLQLIST_ADD_ENTRY(pList_N, pNewEntry);
							break;
						}
					}
					else
					{
						if (!passed)
						{
							pList_N->pFirst = pNewEntry;
							pNewEntry->pNext = pEntry_nList;
							break;
						}
						pPrevEntry_nList->pNext = pNewEntry;
						pNewEntry->pNext = pEntry_nList;
						break;
					}
				}
				pPrevEntry_nList = pEntry_nList;
				pEntry_nList = (T*)pEntry_nList->pNext;
				passed = TRUE;//Ozacava prolazak prvog elementa u listi
			}
		}
		pEntry = (T*)pEntry->pNext;
	 }
	return pList_N;
}

namespace RVL
{
	template <typename Type> struct QList
	{
		Type *pFirst;
		Type **ppNext;
	};

	namespace QLIST
	{
		struct Index
		{
			int Idx;
			Index *pNext;
		};

		struct Index2
		{
			int Idx;
			Index2 *pNext;
			Index2 **pPtrToThis;
		};

		template<typename T> struct Ptr
		{
			T *ptr;
			Ptr<T> *pNext;
		};

		template<typename T> struct Ptr2
		{
			T *ptr;
			Ptr2<T> *pNext;
			Ptr2<T> **pPtrToThis;
		};

		template <typename T> struct Entry
		{
			T data;
			Entry<T> *pNext;
		};

		void CopyToArray(QList<Index> *pList, Array<int> *pArray);

		template<typename T>
		void CopyToArray(QList<T> *pList, Array<T> *pArray)
		{
			T *pData = pList->pFirst;

			T *pData_ = pArray->Element;

			while (pData)
			{
				*(pData_++) = (*pData);

				pData = pData->pNext;
			}

			pArray->n = pData_ - pArray->Element;
		}

		template<typename T>
		void CopyToArray(QList<Entry<T>> *pList, Array<T> *pArray)
		{
			Entry<T> *pEntry = pList->pFirst;

			T *pData = pArray->Element;

			while (pEntry)
			{
				*(pData++) = pEntry->data;

				pEntry = pEntry->pNext;
			}

			pArray->n = pData - pArray->Element;
		}

		template<typename T>
		void CreatePtrArray(QList<T> *pList, Array<T *> *pArray)
		{
			T *pData = pList->pFirst;

			T **ppData_ = pArray->Element;

			while (pData)
			{
				*(ppData_++) = pData;

				pData = pData->pNext;
			}

			pArray->n = ppData_ - pArray->Element;
		}

		template<typename T>
		void InitListArray(Array<QList<T>> &EmptyListArray, Array<QList<T>> &ListArray)
		{
			QList<T> *pEmptyListArrayEnd = EmptyListArray.Element + ListArray.n;

			QList<T> *pList = ListArray.Element;

			QList<T> *pEmptyList;

			for (pEmptyList = EmptyListArray.Element; pEmptyList < pEmptyListArrayEnd; pEmptyList++, pList++)
				RVLQLIST_INIT2(pEmptyList, pList)
		};

		template<typename T>
		int Size(QList<T> &list)
		{
			int size = 0;

			T *pEntry = list.pFirst;

			while (pEntry)
			{
				size++;

				pEntry = pEntry->pNext;
			}

			return size;
		}
	}
}

class CRVLQListArray
{
public:
	CRVLQListArray(void);
	virtual ~CRVLQListArray(void);
	void Init(CRVLMem *pMem = NULL);
	void Reset();
	void InitListArray(RVLQLIST *EmptyListArray,
					   RVLQLIST *ListArray);

public:
	RVLQLIST *m_ListArray;
	int m_Size;

private:
	RVLQLIST *m_EmptyListArray;
};
