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
bool RVL::GetAngleAxis(float *R, float *V, float &theta)
{
	float k = 0.5 * (R[0 * 3 + 0] + R[1 * 3 + 1] + R[2 * 3 + 2] - 1.0);

	if (k > 1.0)
	{
		theta = 0.0;

		return FALSE;
	}
	else if (k < -1.0)
	{
		theta = PI;

		return FALSE;
	}

	theta = acos(k);

	k = 0.5 / sin(theta);

	V[0] = k * (R[2 * 3 + 1] - R[1 * 3 + 2]);
	V[1] = k * (R[0 * 3 + 2] - R[2 * 3 + 0]);
	V[2] = k * (R[1 * 3 + 0] - R[0 * 3 + 1]);

	return TRUE;
}

void RVL::GetDistance(float *t, float &distance)
{
	distance = sqrt(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]);
}

void RVL::PrintMatrix(FILE *fp, double *A, int n, int m)
{
	double *pA = A;

	int i, j;

	for(i = 0; i < n; i++)
	{
		for(j = 0; j < m; j++, pA++)
			fprintf(fp, "%lf\t", *pA);

		fprintf(fp, "\n");
	}
}

void RVL::GetFileNameAndPath(
	char *fileNameWithPath,
	char *&fileName,
	char *&filePath)
{
	fileName = strrchr(fileNameWithPath, '\\') + 1;

	int fileNameLength = (int)strlen(fileName);

	int filePathLength = (int)strlen(fileNameWithPath) - fileNameLength;

	if (filePath)
	{
		if (strlen(filePath) < filePathLength)
		{
			delete[] filePath;

			filePath = new char[filePathLength + 1];
		}
	}
	else
		filePath = new char[filePathLength + 1];

	memcpy(filePath, fileNameWithPath, filePathLength);

	filePath[filePathLength] = '\0';
}

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

ECCVGTLoader::ECCVGTLoader()
{
	nScenes = 0;
	nModels = 0;
	iScene = 0;
	GT.Element = NULL;
	GT.n = 0;
	modelsInDB = NULL;
	GTFolder = NULL;
	GTFilePath = NULL;
}

ECCVGTLoader::~ECCVGTLoader()
{
	int i;

	for (i = 0; i < nScenes; i++)
		RVL_DELETE_ARRAY(GT.Element[i].Element)

	RVL_DELETE_ARRAY(GT.Element);
	RVL_DELETE_ARRAY(modelsInDB);
	RVL_DELETE_ARRAY(GTFolder);
	RVL_DELETE_ARRAY(GTFilePath);
}

bool ECCVGTLoader::Init(char *filePath, char *GTFolderPath, char *modelsID)
{
	nScenes = 1;

	GT.Element = new Array<GTInstance>[nScenes];
	GT.n = nScenes;

	modelsInDB = new char[strlen(modelsID) + 1];
	memcpy(modelsInDB, modelsID, strlen(modelsID));
	modelsInDB[strlen(modelsID)] = '\0';

	GTFolder = new char[strlen(GTFolderPath) + 1];
	memcpy(GTFolder, GTFolderPath, strlen(GTFolderPath));
	GTFolder[strlen(GTFolderPath)] = '\0';

	GTFilePath = new char[200];

	CreateGTFilePath(filePath, GTFilePath);

	if (LoadModels(GTFilePath))
	{
		iScene++;

		return 1;
	}
	else
		return 0;
}

bool ECCVGTLoader::Init(FileSequenceLoader sceneSequence, char *GTFolderPath, char *modelsID)
{
	if (GTFolderPath == NULL)
		return false;

	nScenes = sceneSequence.nFileNames;

	GT.Element = new Array<GTInstance>[nScenes];
	GT.n = nScenes;

	char filePath[200];

	modelsInDB = new char[strlen(modelsID) + 1];
	memcpy(modelsInDB, modelsID, strlen(modelsID));
	modelsInDB[strlen(modelsID)] = '\0';

	GTFolder = new char[strlen(GTFolderPath) + 1];
	memcpy(GTFolder, GTFolderPath, strlen(GTFolderPath));
	GTFolder[strlen(GTFolderPath)] = '\0';

	GTFilePath = new char[200];

	while (sceneSequence.GetNextPath(filePath))
	{
		CreateGTFilePath(filePath, GTFilePath);

		if (!LoadModels(GTFilePath))
			return false;

		iScene++;
	}

	return true;
}

bool ECCVGTLoader::LoadModels(char *filePath)
{
	FILE *fp = fopen(filePath, "r");

	int nSModels;

	if (fp)
	{
		char line[200];
		int iModel, i;

		fgets(line, 200, fp);
		fscanf(fp, "%d\n", &nSModels);
		fgets(line, 200, fp);

		GT.Element[iScene].Element = new GTInstance[nSModels];
		GT.Element[iScene].n = nSModels;

		nModels += nSModels;

		GTInstance *pGT = GT.Element[iScene].Element;
		

		for (iModel = 0; iModel < nSModels; iModel++)
		{
			pGT->iScene = iScene;

			fgets(line, 200, fp);

			line[strlen(line) - 1] = '\0';

			pGT->iModel = FindModelID(line);

			for (i = 0; i < 3; i++)
				fscanf(fp, "%f %f %f %f\n", &pGT->R[i * 3], &pGT->R[i * 3 + 1], &pGT->R[i * 3 + 2], &pGT->t[i]);

			pGT->matched = false;

			fgets(line, 200, fp);
			fgets(line, 200, fp);

			pGT++;
		}

		fclose(fp);

		return 1;
	}
	else
		return 0;
}

int ECCVGTLoader::FindModelID(char *modelName)
{
	//char *dbFileName = new char[50];
	char dbFileName[50];
	int index = 0;
	int ID;

	FileSequenceLoader dbLoader;
	dbLoader.Init(modelsInDB);

	while (dbLoader.GetNextName(dbFileName))
	{
		if (!strcmp(modelName, dbFileName))
		{
			dbLoader.GetID(index, &ID);
			return ID;
		}
		index++;
	}

	return -1;	
}

void ECCVGTLoader::CreateGTFilePath(char *scenePath, char *GTFilePath)
{
	int GTFileSize = strlen(GTFolder);

	int modelNameSize = strlen(strrchr(scenePath, '\\'));

	memcpy(GTFilePath, GTFolder, GTFileSize);

	memcpy(GTFilePath + GTFileSize, scenePath + strlen(scenePath) - modelNameSize, modelNameSize - 3);

	GTFileSize += modelNameSize - 3;

	memcpy(GTFilePath + GTFileSize, "txt", 3);

	GTFileSize += 3;

	memcpy(GTFilePath + GTFileSize, "\0", 1);
}

bool ECCVGTLoader::SaveGTFile(char *filePath)
{
	FILE *fp;

	fp = fopen(filePath, "w");

	int iS, iM, i;

	if (fp)
	{
		GTInstance *pGT;

		for (iS = 0; iS < nScenes; iS++)
		{
			pGT = GT.Element[iS].Element;

			for (iM = 0; iM < GT.Element[iS].n; iM++)
			{
				fprintf(fp, "%d\t%d\t", pGT->iScene, pGT->iModel);

				for (i = 0; i < 9; i++)
					fprintf(fp, "%f\t", pGT->R[i]);

				for (i = 0; i < 3; i++)
					fprintf(fp, "%f\t", pGT->t[i]);

				fprintf(fp, "%d\n", (int)pGT->matched);

				pGT++;
			}
		}

		fclose(fp);

		return 1;
	}
	else
		return 0;
}

void ECCVGTLoader::ResetMatchFlag()
{
	GTInstance *pGT;

	int iModel, nModels;

	for (iScene = 0; iScene < nScenes; iScene++)
	{
		pGT = GT.Element[iScene].Element;

		nModels = GT.Element[iScene].n;

		for (iModel = 0; iModel < nModels; iModel++)
		{
			pGT->matched = false;

			pGT++;
		}
	}
}
//END VIDOVIC
