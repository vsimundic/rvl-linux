#include "RVLPlatform.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "opencv2\opencv.hpp"
#define _CRT_SECURE_NO_WARNINGS 
#include "RVLConst.h"
#ifndef ushort
#define ushort unsigned short int
#endif
#include "RVLArray.h"
#include "RVLKinect.h"
#include "RVLMem.h"
#include "RVLQListArray.h"
#include "Util.h"

using namespace RVL;

void RVL::QuickSort(int *Key, int *Index, int n)
{
	// get range

	int *pKey = Key;

	int min = *(pKey++);
	int max = min;

	int i;

	for (i = 1; i < n; i++, pKey++)
	{
		if (*pKey < min)
			min = *pKey;
		else if (*pKey > max)
			max = *pKey;
	}

	int nBins = max - min + 1;

	// create lookup table

	Array<QList<QLIST::Index>> KeyLT;

	KeyLT.Element = new QList<QLIST::Index>[nBins];
	KeyLT.n = nBins;

	QList<QLIST::Index> *pKeyList;

	for (i = 0; i < nBins; i++)
	{
		pKeyList = KeyLT.Element + i;

		RVLQLIST_INIT(pKeyList);
	}

	QLIST::Index *KeyMem = new QLIST::Index[n];

	QLIST::Index *pKeyLTEntry = KeyMem;

	int j;

	for (i = 0; i < n; i++)
	{
		j = Key[i] - min;

		pKeyList = KeyLT.Element + j;

		RVLQLIST_ADD_ENTRY(pKeyList, pKeyLTEntry);

		pKeyLTEntry->Idx = i;

		pKeyLTEntry++;
	}

	// fill index array

	int *pIndex = Index;

	for (i = 0; i < nBins; i++)
	{
		pKeyList = KeyLT.Element + i;

		pKeyLTEntry = pKeyList->pFirst;

		while (pKeyLTEntry)
		{
			*(pIndex++) = pKeyLTEntry->Idx;

			pKeyLTEntry = pKeyLTEntry->pNext;
		}
	}

	// deallocate lookup table

	delete[] KeyLT.Element;
	delete[] KeyMem;
}

void RVL::RandomColor(unsigned char *color)
{
	color[0] = (unsigned char)(rand() % 256);
	color[1] = (unsigned char)(rand() % 256);
	color[2] = (unsigned char)(rand() % 256);
}

//VIDOVIC
FileSequenceLoader::FileSequenceLoader()
{
	nFileNames = 0;
	currentID = 0;
}

FileSequenceLoader::~FileSequenceLoader()
{
}

bool FileSequenceLoader::Init(char *sequenceFileName)
{
	FILE *fp = fopen(sequenceFileName, "r");

	if (fp)
	{

		char line[200];
		int lineCnt = 0;
		int sequenceFileNameLength;

		char *tabChar;
		char ID[5];

		while (TRUE)
		{
			fgets(line, 100, fp);

			//int linelen = strlen(line);

			if (line[0] == '\n')
				continue;

			if (strstr(line, "end") == line)
			{
				nFileNames = lineCnt;
				break;
			}


			lineCnt++;

			tabChar = strrchr(line, '\t');

			if (tabChar)
			{
				IDs.resize(lineCnt);

				strncpy(ID, line, strlen(line) - strlen(tabChar));

				ID[strlen(line) - strlen(tabChar)] = '\0';

				IDs[lineCnt - 1] = atoi(ID);

				strncpy(line, line + strlen(line) - strlen(tabChar) + 1, strlen(tabChar) - 1);

				line[strlen(tabChar) - 1] = '\0';
			}

			//Save model name
			names.resize(lineCnt, std::vector<char>(0));

			if (strrchr(line, '\\'))
				names[lineCnt - 1].assign(line + strlen(line) - strlen(strrchr(line, '\\')) + 1, line + strlen(line) - 1);
			else
				names[lineCnt - 1].assign(line, line + strlen(line) - 1);

			names[lineCnt - 1].insert(names[lineCnt - 1].end(), 1, '\0');


			//Save path
			paths.resize(lineCnt, std::vector<char>(0));

			paths[lineCnt - 1].assign(line, line + strlen(line) - 1);

			if (line[1] != ':')
			{
				sequenceFileNameLength = strlen(strrchr(sequenceFileName, '\\')) - 1;

				paths[lineCnt - 1].insert(paths[lineCnt - 1].begin(), sequenceFileName, sequenceFileName + strlen(sequenceFileName) - sequenceFileNameLength);
			}

			paths[lineCnt - 1].insert(paths[lineCnt - 1].end(), 1, '\0');

		}

		fclose(fp);

		currentID = 0;

		return 1;
	}
	else
		return 0;
}

bool FileSequenceLoader::Get(int index, char *filePath, char *fileName, int *ID)
{
	if (index < nFileNames)
	{
		GetFilePath(index, filePath);

		GetFileName(index, fileName);

		GetID(index, ID);

		return 1;
	}
	else
		return 0;
}

bool FileSequenceLoader::GetNext(char *filePath, char *fileName, int *ID)
{
	if (nFileNames > 0 && currentID < nFileNames)
	{
		GetFilePath(currentID, filePath);

		GetFileName(currentID, fileName);

		GetID(currentID, ID);

		currentID++;

		return 1;
	}
	else
		return 0;
}

bool FileSequenceLoader::Get(int index, char *filePath, char *fileName)
{
	if (index < nFileNames)
	{
		GetFilePath(index, filePath);

		GetFileName(index, fileName);

		return 1;
	}
	else
		return 0;
}

bool FileSequenceLoader::GetNext(char *filePath, char *fileName)
{
	if (nFileNames > 0 && currentID < nFileNames)
	{
		GetFilePath(currentID, filePath);

		GetFileName(currentID, fileName);

		currentID++;

		return 1;
	}
	else
		return 0;
}

bool FileSequenceLoader::GetFilePath(int index, char *filePath)
{
	if (index < nFileNames)
	{
		char *filePath_;
		int filePathLength;

		filePath_ = paths[index].data();

		filePathLength = paths[index].size();

		memcpy(filePath, filePath_, filePathLength);

		return 1;
	}
	else
		return 0;
}

bool FileSequenceLoader::GetNextPath(char *filePath)
{
	if (nFileNames > 0 && currentID < nFileNames)
	{
		GetFilePath(currentID, filePath);

		currentID++;

		return 1;
	}
	else
		return 0;
}

bool FileSequenceLoader::GetFileName(int index, char *fileName)
{
	if (index < nFileNames)
	{
		char *fileName_;
		int fileNameLength;

		fileName_ = names[index].data();

		fileNameLength = names[index].size();

		memcpy(fileName, fileName_, fileNameLength);

		return 1;
	}
	else
		return 0;
}

bool FileSequenceLoader::GetNextName(char *fileName)
{
	if (nFileNames > 0 && currentID < nFileNames)
	{
		GetFileName(currentID, fileName);

		currentID++;

		return 1;
	}
	else
		return 0;
}

bool FileSequenceLoader::GetID(int index, int *ID)
{
	if (index < nFileNames && IDs.size() > 0)
	{
		*ID = IDs[index];

		return 1;
	}
	else
		return 0;
}

bool FileSequenceLoader::GetNextID(int *ID)
{
	if (nFileNames > 0 && currentID < nFileNames && IDs.size() > 0)
	{
		GetID(currentID, ID);

		currentID++;

		return 1;
	}
	else
		return 0;
}

int FileSequenceLoader::GetLastModelID()
{
	if (IDs.size())
		return IDs[IDs.size() - 1];
	else
		return -1;
}

void FileSequenceLoader::ResetID()
{
	currentID = 0;
}

void FileSequenceLoader::AddModel(int ID, char *filePath, char *fileName)
{
	//Save ID
	IDs.resize(IDs.size() + 1);
	IDs[IDs.size() - 1] = ID;

	//Save model path
	paths.resize(paths.size() + 1, std::vector<char>(0));
	paths[paths.size() - 1].assign(filePath, filePath + strlen(filePath));
	paths[paths.size() - 1].insert(paths[paths.size() - 1].end(), 1, '\0');

	//Save model name
	names.resize(names.size() + 1, std::vector<char>(0));
	names[names.size() - 1].assign(fileName, fileName + strlen(fileName));
	names[names.size() - 1].insert(names[names.size() - 1].end(), 1, '\0');

	nFileNames++;

}
//END VIDOVIC
