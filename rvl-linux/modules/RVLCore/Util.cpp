#include "RVLCore2.h"
#ifdef RVLVTK
#include "RVLVTK.h"
#include <vtkTriangle.h>
#include "vtkPLY.h"
#include "vtkStringArray.h"
#endif
#include "Util.h"
#ifdef RVLVTK
#include "MarchingCubes.h"
#endif

#ifdef RVLLINUX
#include <Eigen/Eigenvalues>
#define RVLFILEPATH_SEPARATOR '/'
#else
#include <Eigen\Eigenvalues>
#define RVLFILEPATH_SEPARATOR '\\'
#endif

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

// VIDOVIC
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
    RVLNORM3(V, k);

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

    for (i = 0; i < n; i++)
    {
        for (j = 0; j < m; j++, pA++)
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

void RVL::LoadCameraParametersFromFile(
    char *cfgFileName,
    Camera &camera,
    CRVLMem *pMem)
{
    CRVLParameterList paramList;
    paramList.m_pMem = pMem;
    RVLPARAM_DATA *pParamData;
    paramList.Init();
    pParamData = paramList.AddParam("Camera.fu", RVLPARAM_TYPE_FLOAT, &(camera.fu));
    pParamData = paramList.AddParam("Camera.fv", RVLPARAM_TYPE_FLOAT, &(camera.fv));
    pParamData = paramList.AddParam("Camera.uc", RVLPARAM_TYPE_FLOAT, &(camera.uc));
    pParamData = paramList.AddParam("Camera.vc", RVLPARAM_TYPE_FLOAT, &(camera.vc));
    pParamData = paramList.AddParam("Camera.w", RVLPARAM_TYPE_INT, &(camera.w));
    pParamData = paramList.AddParam("Camera.h", RVLPARAM_TYPE_INT, &(camera.h));
    paramList.LoadParams(cfgFileName);
}

void RVL::ReadLine(
    FILE *fp,
    int nCharacters,
    char *line)
{
    fgets(line, nCharacters, fp);
    if (line[strlen(line) - 1] == 10)
        line[strlen(line) - 1] = '\0';
    if (line[strlen(line) - 1] == 13)
        line[strlen(line) - 1] = '\0';
}

int RVL::FurthestPoint(
    float *P,
    Array<Vector3<float>> points,
    float *V)
{
    float maxDist = 0.0f;
    float dist;
    Vector3<float> *pPt;
    float dP[3], P_[3];
    int iFurthestPt;
    float fTmp;
    for (int i = 0; i < points.n; i++)
    {
        pPt = points.Element + i;
        RVLDIF3VECTORS(pPt->Element, P, dP);
        if (V)
        {
            fTmp = RVLDOTPRODUCT3(V, dP);
            RVLSCALE3VECTOR(V, fTmp, P_);
            RVLDIF3VECTORS(pPt->Element, P_, dP);
        }
        dist = RVLDOTPRODUCT3(dP, dP);
        if (dist > maxDist)
        {
            maxDist = dist;
            iFurthestPt = i;
        }
    }
    return iFurthestPt;
}

void RVL::CreateConvexTemplate6(float *A)
{
    memset(A, 0, 6 * 3 * sizeof(float));
    for (int i = 0; i < 3; i++)
    {
        A[3 * i + i] = 1.0f;
        A[3 * i + i + 3 * 3] = -1.0f;
    }
}

void RVL::CreateConvexTemplate18(float *A)
{
    CreateConvexTemplate6(A);
    memset(A + 3 * 6, 0, 12 * 3 * sizeof(float));
    int k = 6;
    float cs;
    float *A1, *A2, *A_;
    float fTmp;
    for (int i = 0; i < 6; i++)
    {
        A1 = A + 3 * i;
        for (int j = 0; j < i; j++)
        {
            A2 = A + 3 * j;
            cs = RVLDOTPRODUCT3(A1, A2);
            if (RVLABS(cs) < 0.5f)
            {
                A_ = A + 3 * k;
                RVLSUM3VECTORS(A1, A2, A_);
                RVLNORM3(A_, fTmp);
                k++;
            }
        }
    }
}

void RVL::CreateConvexTemplate66(float *A)
{
    float h = 0.25f * PI;
    float q = 0.5f * h;
    float sh = sin(h);
    float ch = cos(h);
    float sq = sin(q);
    float cq = cos(q);

    float *NT = new float[3 * 13];

    float *N;

    N = NT;
    RVLSET3VECTOR(N, 0.0f, 0.0f, 1.0f);
    N = NT + 3;
    RVLSET3VECTOR(N, 0.0f, -ch, ch);
    N = NT + 2 * 3;
    RVLSET3VECTOR(N, ch, 0.0f, ch);
    N = NT + 11 * 3;
    RVLSET3VECTOR(N, 0.0f, ch, ch);
    N = NT + 12 * 3;
    RVLSET3VECTOR(N, -ch, 0.0f, ch);

    int templ[] = {
        3, 0, 1,
        4, 0, 2,
        5, 1, 2,
        6, 0, 11,
        7, 0, 12,
        8, 2, 11,
        9, 1, 12,
        10, 11, 12};

    int i;
    float *N_, *N__;
    float fTmp;

    for (i = 0; i < 8; i++)
    {
        N = NT + 3 * templ[3 * i];
        N_ = NT + 3 * templ[3 * i + 1];
        N__ = NT + 3 * templ[3 * i + 2];
        RVLSUM3VECTORS(N_, N__, N);
        RVLNORM3(N, fTmp);
    }

    float R[] = {
        0.0f, 0.0f, -1.0f,
        1.0f, 0.0f, 0.0f,
        0.0f, -1.0f, 0.0f};

    float R_[9];

    RVLMXMUL3X3(R, R, R_);

    int j;
    int i_;

    for (i = 0; i < 6; i++)
    {
        for (j = 0; j < 11; j++)
        {
            N = A + 3 * (11 * i + j);

            N_ = NT + 3 * j;

            i_ = i % 3;

            if (i_ == 0)
            {
                RVLCOPY3VECTOR(N_, N);
            }
            else if (i_ == 1)
            {
                RVLMULMX3X3VECT(R, N_, N)
            }
            else
            {
                RVLMULMX3X3VECT(R_, N_, N)
            }

            if (i >= 3)
            {
                RVLNEGVECT3(N, N);
            }
        }
    }

    delete[] NT;
}

void RVL::CreateTemplateLookUpTable(
    Array2D<float> A,
    int resolution,
    ConvexTemplateLookUpTable &CTLUT)
{
    CTLUT.LUT.a = CTLUT.LUT.b = CTLUT.LUT.c = resolution + 1;

    RVL_DELETE_ARRAY(CTLUT.LUT.Element);

    int nLUTElements = CTLUT.LUT.a * CTLUT.LUT.b * CTLUT.LUT.c;

    CTLUT.LUT.Element = new int[nLUTElements];

    float securityMargin = 1.1f;

    CTLUT.quant = securityMargin * 2.0f / (float)resolution;

    CTLUT.iHalfRange = resolution / 2;

    CTLUT.halfRange = (float)(CTLUT.iHalfRange) + 0.5f;

    FILE *fp = fopen("..\\convexTemplateLUT.dat", "rb");

    if (fp)
        fread(CTLUT.LUT.Element, sizeof(float), nLUTElements, fp);
    else
    {
        int i, j, k, l, iClosest;
        float x[3];
        float fTmp, cs, maxcs;
        float *a;

        for (k = 0; k <= resolution; k++)
            for (j = 0; j <= resolution; j++)
                for (i = 0; i <= resolution; i++)
                {
                    x[0] = CTLUT.quant * (float)(i - CTLUT.iHalfRange);
                    x[1] = CTLUT.quant * (float)(j - CTLUT.iHalfRange);
                    x[2] = CTLUT.quant * (float)(k - CTLUT.iHalfRange);

                    fTmp = sqrt(RVLDOTPRODUCT3(x, x));

                    if (fTmp > 1e-10)
                    {
                        RVLSCALE3VECTOR2(x, fTmp, x);

                        maxcs = -2.0f;

                        for (l = 0; l < A.h; l++)
                        {
                            a = A.Element + 3 * l;

                            cs = RVLDOTPRODUCT3(a, x);

                            if (cs > maxcs)
                            {
                                maxcs = cs;

                                iClosest = l;
                            }
                        }
                    }
                    else
                        iClosest = 0;

                    CTLUT.LUT.Element[RVL3DARRAY_ELEMENT_INDEX(CTLUT.LUT, i, j, k)] = iClosest;
                }

        fp = fopen("convexTemplateLUT.dat", "wb");

        fwrite(CTLUT.LUT.Element, sizeof(float), nLUTElements, fp);
    }

    fclose(fp);
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
            // fgets(line, 200, fp);
            ReadLine(fp, 200, line);

            if (line[0] == '\n')
                continue;

            if (strlen(line) == 0)
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

            // Save model name
            names.resize(lineCnt, std::vector<char>(0));

            if (strrchr(line, RVLFILEPATH_SEPARATOR))
                // names[lineCnt - 1].assign(line + strlen(line) - strlen(strrchr(line, RVLFILEPATH_SEPARATOR)) + 1, line + strlen(line) - 1);
                names[lineCnt - 1].assign(line + strlen(line) - strlen(strrchr(line, RVLFILEPATH_SEPARATOR)) + 1, line + strlen(line));
            else
                // names[lineCnt - 1].assign(line, line + strlen(line) - 1);
                names[lineCnt - 1].assign(line, line + strlen(line));

            names[lineCnt - 1].insert(names[lineCnt - 1].end(), 1, '\0');

            // Save path
            paths.resize(lineCnt, std::vector<char>(0));

            // paths[lineCnt - 1].assign(line, line + strlen(line) - 1);
            paths[lineCnt - 1].assign(line, line + strlen(line));

            if (line[1] != ':')
            {
                sequenceFileNameLength = strlen(strrchr(sequenceFileName, RVLFILEPATH_SEPARATOR)) - 1;

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
    // Save ID
    IDs.resize(IDs.size() + 1);
    IDs[IDs.size() - 1] = ID;

    // Save model path
    paths.resize(paths.size() + 1, std::vector<char>(0));
    paths[paths.size() - 1].assign(filePath, filePath + strlen(filePath));
    paths[paths.size() - 1].insert(paths[paths.size() - 1].end(), 1, '\0');

    // Save model name
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
    bRotInvariantModel = NULL;
    dataSetFlag = RVL_DATASET_FLAG_TUW_KINECT;
}

ECCVGTLoader::~ECCVGTLoader()
{
    if (GT.n > 0 && GT.Element)
    {
        int i;

        for (i = 0; i < nScenes; i++)
            RVL_DELETE_ARRAY(GT.Element[i].Element)

        RVL_DELETE_ARRAY(GT.Element);
    }
    RVL_DELETE_ARRAY(modelsInDB);
    RVL_DELETE_ARRAY(GTFolder);
    RVL_DELETE_ARRAY(GTFilePath);
    RVL_DELETE_ARRAY(bRotInvariantModel);
}

bool ECCVGTLoader::Init(char *filePath, char *GTFolderPath, char *modelsID, DWORD dataSet)
{
    if (dataSet)
        dataSetFlag = dataSet;

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

    // Kinect dataset
    if (dataSetFlag == RVL_DATASET_FLAG_TUW_KINECT)
        CreateGTFilePath(filePath, GTFilePath);
    // Willow and Challenge dataset
    else if (dataSetFlag == RVL_DATASET_FLAG_WILLOW_AND_CHALLENGE)
        CreateWillowAndChallengeGTFilePath(filePath, GTFilePath);

    if (LoadModels(GTFilePath))
    {
        iScene++;

        return 1;
    }
    else
        return 0;
}

bool ECCVGTLoader::Init(FileSequenceLoader sceneSequence, char *GTFolderPath, char *modelsID, DWORD dataSet)
{
    if (GTFolderPath == NULL)
    {
        printf("Cannot find a GT file!\n");

        return false;
    }

    if (dataSet)
        dataSetFlag = dataSet;

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
        // Kinect dataset
        if (dataSetFlag == RVL_DATASET_FLAG_TUW_KINECT)
            CreateGTFilePath(filePath, GTFilePath);
        // Willow and Challenge dataset
        else if (dataSetFlag == RVL_DATASET_FLAG_WILLOW_AND_CHALLENGE)
            CreateWillowAndChallengeGTFilePath(filePath, GTFilePath);
        // ICL dataset
        else if (dataSetFlag == RVL_DATASET_FLAG_ICL)
            CreateGTFilePath(filePath, GTFilePath);

        if (!LoadModels(GTFilePath))
            return false;

        iScene++;
    }

    FILE *fp = fopen((std::string(GTFolderPath) + "\\gt.dat").data(), "w");

    int nScenes = iScene;

    int iObject = 0;

    Array<GTInstance> *pGTArray;

    for (iScene = 0; iScene < nScenes; iScene++)
    {
        pGTArray = GT.Element + iScene;

        GTInstance *pGT;
        int iGT;

        for (iGT = 0; iGT < pGTArray->n; iGT++)
        {
            pGT = pGTArray->Element + iGT;

            fprintf(fp, "%d\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\n", iObject, iScene, pGT->iModel, pGT->t[0], pGT->t[1], pGT->t[2], pGT->R[2], pGT->R[5], pGT->R[8]);

            iObject++;
        }
    }

    fclose(fp);

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

        // fgets(line, 200, fp);
        ReadLine(fp, 200, line);
        fscanf(fp, "%d\n", &nSModels);
        // fgets(line, 200, fp);
        ReadLine(fp, 200, line);

        GT.Element[iScene].Element = new GTInstance[nSModels];
        GT.Element[iScene].n = nSModels;

        nModels += nSModels;

        GTInstance *pGT = GT.Element[iScene].Element;

        for (iModel = 0; iModel < nSModels; iModel++)
        {
            pGT->iScene = iScene;

            // fgets(line, 200, fp);
            ReadLine(fp, 200, line);

            line[strlen(line)] = '\0';

            pGT->iModel = FindModelID(line);

            for (i = 0; i < 3; i++)
                fscanf(fp, "%f %f %f %f\n", &pGT->R[i * 3], &pGT->R[i * 3 + 1], &pGT->R[i * 3 + 2], &pGT->t[i]);

            if (dataSetFlag == RVL_DATASET_FLAG_TUW_KINECT)
                RVLSCALEMX3X3(pGT->R, 1000.0f, pGT->R); // This is for TUW Kinect dataset only!

            pGT->matched = false;

            // fgets(line, 200, fp);
            // fgets(line, 200, fp);
            ReadLine(fp, 200, line);
            ReadLine(fp, 200, line);

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
    // char *dbFileName = new char[50];
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

void ECCVGTLoader::CreateWillowAndChallengeGTFilePath(char *scenePath, char *GTFilePath)
{
    int GTFileSize = strlen(GTFolder);

    int modelNameSize = strlen(strrchr(scenePath, '\\')) + strlen("\\T_xx");

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

void ECCVGTLoader::LoadRotInvarianceInfo(char *rotInvarianceFileName)
{
    if (rotInvarianceFileName == NULL)
    {
        printf("Rotation invariance model info is not available.\n");

        return;
    }

    FILE *fp = fopen(rotInvarianceFileName, "r");

    if (fp == NULL)
    {
        printf("Rotation invariance model info is not available.\n");

        return;
    }

    printf("Loading rotation invariance model info.\n");

    fscanf(fp, "%d\n", &nModelsInDB);

    RVL_DELETE_ARRAY(bRotInvariantModel);

    bRotInvariantModel = new bool[nModelsInDB];

    memset(bRotInvariantModel, 0, nModelsInDB * sizeof(bool));

    int iModel;

    while (!feof(fp))
    {
        fscanf(fp, "%d\n", &iModel);

        if (iModel >= 0 && iModel < nModelsInDB)
            bRotInvariantModel[iModel] = true;
    }

    fclose(fp);
}

void RVL::TransformWillowAndChallengeGT2ECCVFormat(char *sceneGTPath)
{
    printf("Willow and Challenge GT files formating...\n");

    FileSequenceLoader sceneGTSequence;
    sceneGTSequence.Init(sceneGTPath);

    char sceneGTFilePath[200];
    char sceneFileName[200];
    char modelName[50];
    char *modelName_;
    char *sceneGTFilePath_;
    char line[500];
    char *ptr;

    int nModels;

    float T[16];

    FILE *fpSceneGT;
    FILE *fpNewSceneGT;

    while (sceneGTSequence.GetNextPath(sceneGTFilePath))
    {
        sceneGTFilePath_ = RVLCreateFileName(sceneGTFilePath, ".anno", -1, ".txt");

        // extract scene fileName from path
        ptr = strrchr(sceneGTFilePath, '\\');

        if (ptr)
        {
            memcpy(sceneFileName, ptr + 1, sizeof(char) * (strlen(ptr) - 1));
            memcpy(sceneFileName + (strlen(ptr) - strlen(".anno") - 1), ".ply\0", 5);
        }

        nModels = 0;

        fpSceneGT = fopen(sceneGTFilePath, "r");
        if (fpSceneGT)
        {
            while (!feof(fpSceneGT))
            {
                if (fgets(line, 500, fpSceneGT))
                    nModels++;
            }

            rewind(fpSceneGT);

            fpNewSceneGT = fopen(sceneGTFilePath_, "w");
            fprintf(fpNewSceneGT, "%s\n", sceneFileName);
            fprintf(fpNewSceneGT, "%d\n", nModels);
            fprintf(fpNewSceneGT, "%.1f\n", 0.0);

            for (int iModel = 0; iModel < nModels; iModel++)
            {
                fgets(line, 500, fpSceneGT);

                sscanf(line, "%s %*s %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f", modelName, &T[0], &T[1], &T[2], &T[3], &T[4], &T[5], &T[6], &T[7], &T[8], &T[9], &T[10], &T[11], &T[12], &T[13], &T[14], &T[15]);

                modelName_ = RVLCreateFileName(modelName, "", -1, ".ply");

                fprintf(fpNewSceneGT, "%s\n", modelName_);
                fprintf(fpNewSceneGT, "%f %f %f %f\n", T[0], T[1], T[2], T[3]);
                fprintf(fpNewSceneGT, "%f %f %f %f\n", T[4], T[5], T[6], T[7]);
                fprintf(fpNewSceneGT, "%f %f %f %f\n", T[8], T[9], T[10], T[11]);
                fprintf(fpNewSceneGT, "%d %d %d %d\n\n", (int)T[12], (int)T[13], (int)T[14], (int)T[15]);
            }
        }

        fclose(fpSceneGT);
        fclose(fpNewSceneGT);
    }

    printf("Willow and Challenge GT files formating FINISHED!\n");

    //	printf("Willow and Challenge GT files formating...\n");
    //	char sceneFileName[50];
    //	char sceneName[50];
    //	char sceneGTFilePath[200];
    //	char sceneGTFilePath_[200];
    //	char newSceneGTFilePath[200];
    //	char sceneFilePath[200];
    //	//char *sceneGroupName; //T_01, T_02, ...
    //	char *modelName;
    //	int nModelsPerScene;
    //	int sceneID = 0;
    //	float T[16];
    //	int stringLen;
    //
    //	FileSequenceLoader sceneGTSequence;
    //	sceneGTSequence.Init(sceneGTPath);
    //
    //	FILE *fpSceneGT;
    //	FILE *fpNewSceneGT;
    //
    //	while (!feof(fpModelsPerScene))
    //	{
    //
    //		fscanf(fpModelsPerScene, "%d\n", &nModelsPerScene);
    //		sceneSequence.GetNextName(sceneFileName);
    //
    //		memcpy(sceneName, sceneFileName, sizeof(char) * (strlen(sceneFileName) - strlen(".ply")));
    //		memcpy(sceneName + strlen(sceneFileName) - strlen(".ply"), "\0", 1);
    //
    //		/*sceneSequence.GetFilePath(sceneID, sceneFilePath);
    //		sceneGroupName = strstr(sceneFilePath, "T_");
    //		memcpy(sceneGroupName, sceneGroupName, sizeof(char) * strlen("T_xx"));
    //		memcpy(sceneGroupName + sizeof(char) * strlen("T_xx"), "\0", 1);*/
    //
    //		for (int iModel = 0; iModel < nModelsPerScene; iModel++)
    //		{
    //			sceneGTSequence.GetNextPath(sceneGTFilePath);
    //
    //			//Create object name from GT file path
    //			memcpy(sceneGTFilePath_, sceneGTFilePath, strlen(sceneGTFilePath) + 1);
    //			modelName = strstr(sceneGTFilePath_, sceneName);
    //			memcpy(modelName, modelName + strlen(sceneName) + 1, sizeof(char) * (strlen(modelName) - strlen(sceneName)));
    //			memcpy(modelName + (strlen(modelName) - strlen(".txt") - strlen("_0")), ".ply\0", 5);
    //
    //			stringLen = strlen(sceneGTFilePath) - strlen("_") - strlen(modelName) + strlen(".ply") - strlen("_0") - strlen(".txt");
    //
    //			memcpy(newSceneGTFilePath, sceneGTFilePath, sizeof(char) * stringLen);
    //			memcpy(newSceneGTFilePath + stringLen, ".txt\0", 5);
    //
    //			//Read GT values
    //			fpSceneGT = fopen(sceneGTFilePath, "r");
    //			if (fpSceneGT)
    //			{
    //				fscanf(fpSceneGT, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f", &T[0], &T[1], &T[2], &T[3], &T[4], &T[5], &T[6], &T[7], &T[8], &T[9], &T[10], &T[11], &T[12], &T[13], &T[14], &T[15]);
    //				fclose(fpSceneGT);
    //			}
    //
    //
    //			if (iModel == 0)
    //			{
    //				fpNewSceneGT = fopen(newSceneGTFilePath, "w");
    //				fprintf(fpNewSceneGT, "%s\n", sceneFileName);
    //				fprintf(fpNewSceneGT, "%d\n", nModelsPerScene);
    //				fprintf(fpNewSceneGT, "%.1f\n", 0.0);
    //			}
    //			else
    //				fpNewSceneGT = fopen(newSceneGTFilePath, "a");
    //
    //			fprintf(fpNewSceneGT, "%s\n", modelName);
    //			fprintf(fpNewSceneGT, "%f %f %f %f\n", T[0] * scale, T[1] * scale, T[2] * scale, T[3] * scale);
    //			fprintf(fpNewSceneGT, "%f %f %f %f\n", T[4] * scale, T[5] * scale, T[6] * scale, T[7] * scale);
    //			fprintf(fpNewSceneGT, "%f %f %f %f\n", T[8] * scale, T[9] * scale, T[10] * scale, T[11] * scale);
    //			fprintf(fpNewSceneGT, "%d %d %d %d\n\n", (int)T[12], (int)T[13], (int)T[14], (int)T[15]);
    //
    //			fclose(fpNewSceneGT);
    //
    //		}
    //
    //		printf("GT created for scene %d: %s with %d models\n", sceneID, sceneFileName, nModelsPerScene);
    //		printf("\tGT path: %s\n", newSceneGTFilePath);
    //
    //		sceneID++;
    //	}
    //
    //	printf("Willow and Challenge GT files formating FINISHED!\n");
    //
    //	fclose(fpModelsPerScene);
}

void RVL::TransformICLGT2ECCVFormat(char *sceneGTPath)
{
    printf("ICL GT files formating...\n");

    // char *OLD_NAME = "poses";
    char *OLD_NAME = "img_";
    char *NEW_NAME = "img_";
    char *PLY_EXTENSION = ".ply";
    char *TXT_EXTENSION = ".txt";

    char GTPath[200];
    char sceneName[200];
    char modelName[200];
    char sceneGTFilePath[200];
    char sceneGTFilePathNew[200];
    char *pChName, *pChPoint, *pCh;
    char sceneNumberStr[10];

    float T[16];
    int sceneNumber;
    int nModels;

    FILE *fpOld, *fpNew;

    pCh = strrchr(sceneGTPath, '\\');

    int pathLength = strlen(sceneGTPath) - strlen(pCh);
    memcpy(GTPath, sceneGTPath, pathLength);
    GTPath[pathLength] = '\0';

    // Extract model name
    pCh = strrchr(GTPath, '\\');
    memcpy(modelName, pCh + 1, strlen(pCh));
    memcpy(modelName + strlen(pCh) - 1, "_model", strlen("_model"));
    memcpy(modelName + strlen(pCh) + strlen("_model") - 1, PLY_EXTENSION, strlen(PLY_EXTENSION));
    modelName[strlen(pCh) + strlen(PLY_EXTENSION) + strlen("_model") - 1] = '\0';

    FileSequenceLoader sceneGTSequence;
    sceneGTSequence.Init(sceneGTPath);

    while (sceneGTSequence.GetNextPath(sceneGTFilePath))
    {
        pChName = strstr(sceneGTFilePath, OLD_NAME);
        pChName += strlen(OLD_NAME);

        pChPoint = strchr(sceneGTFilePath, '.');

        memcpy(sceneNumberStr, pChName, pChPoint - pChName);
        sceneNumberStr[pChPoint - pChName] = '\0';

        int sceneNumber = atoi(sceneNumberStr);

        // Create new GT file path
        sprintf(sceneGTFilePathNew, "%s%s%s%03d%s", GTPath, "\\ECCV_Format\\", NEW_NAME, sceneNumber, ".txt");

        // Create scene name
        sprintf(sceneName, "%s%03d%s\0", NEW_NAME, sceneNumber, PLY_EXTENSION);

        fpOld = fopen(sceneGTFilePath, "r");
        fpNew = fopen(sceneGTFilePathNew, "w");

        if (fpOld && fpNew)
        {
            fscanf(fpOld, "%d", &nModels);

            fprintf(fpNew, "%s\n", sceneName);
            fprintf(fpNew, "%d\n", nModels);
            fprintf(fpNew, "0.0\n");

            for (int iModel = 0; iModel < nModels; iModel++)
            {
                // load T matrix from the original file
                fscanf(fpOld, "%f,%f,%f,%f\n", &T[0], &T[1], &T[2], &T[3]);
                fscanf(fpOld, "%f,%f,%f,%f\n", &T[4], &T[5], &T[6], &T[7]);
                fscanf(fpOld, "%f,%f,%f,%f\n", &T[8], &T[9], &T[10], &T[11]);
                fscanf(fpOld, "%f,%f,%f,%f\n", &T[12], &T[13], &T[14], &T[15]);

                // write model name to the new file
                fprintf(fpNew, "%s\n", modelName);

                fprintf(fpNew, "%f\t%f\t%f\t%f\n", T[0], T[1], T[2], T[3] / 1000);
                fprintf(fpNew, "%f\t%f\t%f\t%f\n", T[4], T[5], T[6], T[7] / 1000);
                fprintf(fpNew, "%f\t%f\t%f\t%f\n", T[8], T[9], T[10], T[11] / 1000);
                fprintf(fpNew, "%f\t%f\t%f\t%f\n", T[12], T[13], T[14], T[15]);

                fprintf(fpNew, "\n");
            }

            fclose(fpOld);
            fclose(fpNew);
        }
    }

    printf("ICL GT files formating FINISHED!\n");
}

void RVL::TransformCorrectedICLGT2ECCVFormat(char *sceneGTPath)
{
    printf("ICL GT files formating...\n");

    char *OLD_NAME = "poses";
    char *NEW_NAME = "img_";
    char *PLY_EXTENSION = ".ply";
    char *TXT_EXTENSION = ".txt";

    char GTPath[200];
    char sceneName[200];
    char modelName[200];
    char sceneGTFilePath[200];
    char sceneGTFilePathNew[200];
    char *pChName, *pChPoint, *pCh;
    char sceneNumberStr[10];

    float T[16];
    int sceneNumber;
    int nModels;

    FILE *fpOld, *fpNew;

    pCh = strrchr(sceneGTPath, '\\');

    int pathLength = strlen(sceneGTPath) - strlen(pCh);
    memcpy(GTPath, sceneGTPath, pathLength);
    GTPath[pathLength] = '\0';

    // Extract model name
    pCh = strrchr(GTPath, '\\');
    memcpy(modelName, pCh + 1, strlen(pCh));
    memcpy(modelName + strlen(pCh) - 1, "_model", strlen("_model"));
    memcpy(modelName + strlen(pCh) + strlen("_model") - 1, PLY_EXTENSION, strlen(PLY_EXTENSION));
    modelName[strlen(pCh) + strlen(PLY_EXTENSION) + strlen("_model") - 1] = '\0';

    FileSequenceLoader sceneGTSequence;
    sceneGTSequence.Init(sceneGTPath);

    while (sceneGTSequence.GetNextPath(sceneGTFilePath))
    {
        pChName = strstr(sceneGTFilePath, OLD_NAME);
        pChName += strlen(OLD_NAME);

        pChPoint = strchr(sceneGTFilePath, '.');

        memcpy(sceneNumberStr, pChName, pChPoint - pChName);
        sceneNumberStr[pChPoint - pChName] = '\0';

        int sceneNumber = atoi(sceneNumberStr);

        // Create new GT file path
        sprintf(sceneGTFilePathNew, "%s%s%s%03d%s", GTPath, "\\", NEW_NAME, sceneNumber, ".txt");

        // Create scene name
        sprintf(sceneName, "%s%03d%s\0", NEW_NAME, sceneNumber, PLY_EXTENSION);

        fpOld = fopen(sceneGTFilePath, "r");
        fpNew = fopen(sceneGTFilePathNew, "w");

        if (fpOld && fpNew)
        {
            fscanf(fpOld, "%d", &nModels);

            fprintf(fpNew, "%s\n", sceneName);
            fprintf(fpNew, "%d\n", nModels);
            fprintf(fpNew, "0.0\n");

            for (int iModel = 0; iModel < nModels; iModel++)
            {
                // load T matrix from the original file
                fscanf(fpOld, "%f\t%f\t%f\t%f\n", &T[0], &T[1], &T[2], &T[3]);
                fscanf(fpOld, "%f\t%f\t%f\t%f\n", &T[4], &T[5], &T[6], &T[7]);
                fscanf(fpOld, "%f\t%f\t%f\t%f\n", &T[8], &T[9], &T[10], &T[11]);
                fscanf(fpOld, "%f\t%f\t%f\t%f\n", &T[12], &T[13], &T[14], &T[15]);

                // write model name to the new file
                fprintf(fpNew, "%s\n", modelName);

                fprintf(fpNew, "%f\t%f\t%f\t%f\n", T[0], T[1], T[2], T[3]);
                fprintf(fpNew, "%f\t%f\t%f\t%f\n", T[4], T[5], T[6], T[7]);
                fprintf(fpNew, "%f\t%f\t%f\t%f\n", T[8], T[9], T[10], T[11]);
                fprintf(fpNew, "%f\t%f\t%f\t%f\n", T[12], T[13], T[14], T[15]);

                fprintf(fpNew, "\n");
            }

            fclose(fpOld);
            fclose(fpNew);
        }
    }

    printf("ICL GT files formating FINISHED!\n");
}

// END VIDOVIC

namespace RVL
{
#ifdef RVLVTK
    typedef struct _plyVertex
    {
        float x[3]; // the usual 3-space position of a vertex
        unsigned char red;
        unsigned char green;
        unsigned char blue;
        unsigned char alpha;
        float tex[2];
        float n[3];
    } plyVertex;

    typedef struct _plyFace
    {
        unsigned char nverts; // number of vertex indices in list
        int *verts;           // vertex index list
        unsigned char red;
        unsigned char green;
        unsigned char blue;
        unsigned char alpha;
    } plyFace;

    vtkStandardNewMacro(RVLVTKPLYWriter);

    void RVLVTKPLYWriter::WritePolyData(
        std::string fileName,
        vtkSmartPointer<vtkPolyData> pPolygonData)
    {
        SetFileName((char *)(fileName.data()));
        SetColorMode(VTK_COLOR_MODE_DEFAULT);
        SetArrayName("Colors");
        SetInputData(pPolygonData);
        WriteDataWithNormals();
    }

    // Function WriteDataWithNormals is created by modifying function vtkPLYWriter::WriteData of the VTK 8.1.1. library.

    void RVLVTKPLYWriter::WriteDataWithNormals()
    {
        vtkIdType i, j, idx;
        vtkPoints *inPts;
        vtkCellArray *polys;
        vtkPolyData *input = this->GetInput();

#ifdef RVLVTK7_1
        unsigned char *cellColors, *pointColors;
#else
        vtkSmartPointer<vtkUnsignedCharArray> cellColors, pointColors;
#endif
        vtkSmartPointer<vtkFloatArray> pointNormals;
        PlyFile *ply;
        float version;
        static const char *elemNames[] = {"vertex", "face"};
        static PlyProperty vertProps[] = {
            // property information for a vertex
            {"x", PLY_FLOAT, PLY_FLOAT, static_cast<int>(offsetof(plyVertex, x)), 0, 0, 0, 0},
            {"y", PLY_FLOAT, PLY_FLOAT, static_cast<int>(offsetof(plyVertex, x) + sizeof(float)), 0, 0, 0,
             0},
            {"z", PLY_FLOAT, PLY_FLOAT,
             static_cast<int>(offsetof(plyVertex, x) + sizeof(float) + sizeof(float)), 0, 0, 0, 0},
            {"red", PLY_UCHAR, PLY_UCHAR, static_cast<int>(offsetof(plyVertex, red)), 0, 0, 0, 0},
            {"green", PLY_UCHAR, PLY_UCHAR, static_cast<int>(offsetof(plyVertex, green)), 0, 0, 0, 0},
            {"blue", PLY_UCHAR, PLY_UCHAR, static_cast<int>(offsetof(plyVertex, blue)), 0, 0, 0, 0},
            {"alpha", PLY_UCHAR, PLY_UCHAR, static_cast<int>(offsetof(plyVertex, alpha)), 0, 0, 0, 0},
            {(TextureCoordinatesName == 1) ? "texture_u" : "u", PLY_FLOAT, PLY_FLOAT,
             static_cast<int>(offsetof(plyVertex, tex)), 0, 0, 0, 0},
            {(TextureCoordinatesName == 1) ? "texture_v" : "v", PLY_FLOAT, PLY_FLOAT,
             static_cast<int>(offsetof(plyVertex, tex) + sizeof(float)), 0, 0, 0, 0},
            {"nx", PLY_FLOAT, PLY_FLOAT, static_cast<int>(offsetof(plyVertex, n)), 0, 0, 0, 0},
            {"ny", PLY_FLOAT, PLY_FLOAT, static_cast<int>(offsetof(plyVertex, n) + sizeof(float)), 0, 0, 0,
             0},
            {"nz", PLY_FLOAT, PLY_FLOAT,
             static_cast<int>(offsetof(plyVertex, n) + sizeof(float) + sizeof(float)), 0, 0, 0, 0},
        };
        static PlyProperty faceProps[] = {
            // property information for a face
            {"vertex_indices", PLY_INT, PLY_INT, static_cast<int>(offsetof(plyFace, verts)), 1, PLY_UCHAR,
             PLY_UCHAR, static_cast<int>(offsetof(plyFace, nverts))},
            {"red", PLY_UCHAR, PLY_UCHAR, static_cast<int>(offsetof(plyFace, red)), 0, 0, 0, 0},
            {"green", PLY_UCHAR, PLY_UCHAR, static_cast<int>(offsetof(plyFace, green)), 0, 0, 0, 0},
            {"blue", PLY_UCHAR, PLY_UCHAR, static_cast<int>(offsetof(plyFace, blue)), 0, 0, 0, 0},
            {"alpha", PLY_UCHAR, PLY_UCHAR, static_cast<int>(offsetof(plyFace, alpha)), 0, 0, 0, 0},
        };

        // Get input and check data
        polys = input->GetPolys();
        inPts = input->GetPoints();
        if (inPts == nullptr || polys == nullptr)
        {
            vtkErrorMacro(<< "No data to write!");
            return;
        }

        if (this->FileName == nullptr)
        {
            vtkErrorMacro(<< "Please specify FileName to write");
            return;
        }

        // Open the file in appropriate way
        if (this->FileType == VTK_BINARY)
        {
            if (this->DataByteOrder == VTK_LITTLE_ENDIAN)
            {
                ply = vtkPLY::ply_open_for_writing(this->FileName, 2, elemNames, PLY_BINARY_LE, &version);
            }
            else
            {
                ply = vtkPLY::ply_open_for_writing(this->FileName, 2, elemNames, PLY_BINARY_BE, &version);
            }
        }
        else
        {
            ply = vtkPLY::ply_open_for_writing(this->FileName, 2, elemNames,
                                               PLY_ASCII, &version);
        }

        if (ply == nullptr)
        {
            vtkErrorMacro(<< "Error opening PLY file");
            return;
        }

        // compute colors, if any
        vtkIdType numPts = inPts->GetNumberOfPoints();
        vtkIdType numPolys = polys->GetNumberOfCells();
        pointColors = this->GetColors(numPts, input->GetPointData());
        cellColors = this->GetColors(numPolys, input->GetCellData());

#ifndef RVLVTK7_1
        bool pointAlpha = pointColors && pointColors->GetNumberOfComponents() == 4;
        bool cellAlpha = cellColors && cellColors->GetNumberOfComponents() == 4;
#endif

        // get texture coordinates, if any
        const float *textureCoords = this->GetTextureCoordinates(numPts, input->GetPointData());

        // get normals, if any

        vtkDataArray *normalDataArray = input->GetPointData()->GetArray("Normals");
        if (normalDataArray == nullptr)
            pointNormals = vtkArrayDownCast<vtkFloatArray>(input->GetPointData()->GetNormals());
        else
        {
            pointNormals = vtkArrayDownCast<vtkFloatArray>(normalDataArray);
            if (pointNormals == nullptr)
                pointNormals = vtkArrayDownCast<vtkFloatArray>(input->GetPointData()->GetNormals());
        }

        // describe what properties go into the vertex and face elements
        vtkPLY::ply_element_count(ply, "vertex", numPts);
        vtkPLY::ply_describe_property(ply, "vertex", &vertProps[0]);
        vtkPLY::ply_describe_property(ply, "vertex", &vertProps[1]);
        vtkPLY::ply_describe_property(ply, "vertex", &vertProps[2]);
        if (pointColors)
        {
            vtkPLY::ply_describe_property(ply, "vertex", &vertProps[3]);
            vtkPLY::ply_describe_property(ply, "vertex", &vertProps[4]);
            vtkPLY::ply_describe_property(ply, "vertex", &vertProps[5]);
#ifndef RVLVTK7_1
            if (pointAlpha)
            {
                vtkPLY::ply_describe_property(ply, "vertex", &vertProps[6]);
            }
#endif
        }
        if (textureCoords)
        {
            vtkPLY::ply_describe_property(ply, "vertex", &vertProps[7]);
            vtkPLY::ply_describe_property(ply, "vertex", &vertProps[8]);
        }
        if (pointNormals)
        {
            vtkPLY::ply_describe_property(ply, "vertex", &vertProps[9]);
            vtkPLY::ply_describe_property(ply, "vertex", &vertProps[10]);
            vtkPLY::ply_describe_property(ply, "vertex", &vertProps[11]);
        }

        vtkPLY::ply_element_count(ply, "face", numPolys);
        vtkPLY::ply_describe_property(ply, "face", &faceProps[0]);
        if (cellColors)
        {
            vtkPLY::ply_describe_property(ply, "face", &faceProps[1]);
            vtkPLY::ply_describe_property(ply, "face", &faceProps[2]);
            vtkPLY::ply_describe_property(ply, "face", &faceProps[3]);
#ifndef RVLVTK7_1
            if (cellAlpha)
            {
                vtkPLY::ply_describe_property(ply, "face", &faceProps[4]);
            }
#endif
        }

        // write comments and an object information field
        for (idx = 0; idx < this->HeaderComments->GetNumberOfValues(); ++idx)
        {
            vtkPLY::ply_put_comment(ply, this->HeaderComments->GetValue(idx));
        }
        vtkPLY::ply_put_obj_info(ply, "vtkPolyData points and polygons: vtk4.0");

        // complete the header
        vtkPLY::ply_header_complete(ply);

        // set up and write the vertex elements
        plyVertex vert;
        vtkPLY::ply_put_element_setup(ply, "vertex");
        double dpoint[3];
        float normal[3];
        for (i = 0; i < numPts; i++)
        {
            inPts->GetPoint(i, dpoint);
            vert.x[0] = static_cast<float>(dpoint[0]);
            vert.x[1] = static_cast<float>(dpoint[1]);
            vert.x[2] = static_cast<float>(dpoint[2]);
            if (pointColors)
            {
#ifdef RVLVTK7_1
                idx = 3 * i;
                vert.red = *(pointColors + idx);
                vert.green = *(pointColors + idx + 1);
                vert.blue = *(pointColors + idx + 2);
#else
                idx = pointAlpha ? 4 * i : 3 * i;
                vert.red = pointColors->GetValue(idx);
                vert.green = pointColors->GetValue(idx + 1);
                vert.blue = pointColors->GetValue(idx + 2);
                if (pointAlpha)
                {
                    vert.alpha = pointColors->GetValue(idx + 3);
                }
#endif
            }
            if (textureCoords)
            {
                idx = 2 * i;
                vert.tex[0] = *(textureCoords + idx);
                vert.tex[1] = *(textureCoords + idx + 1);
            }
            if (pointNormals)
            {
                pointNormals->GetTypedTuple(i, normal);
                vert.n[0] = normal[0];
                vert.n[1] = normal[1];
                vert.n[2] = normal[2];
            }
            vtkPLY::ply_put_element(ply, (void *)&vert);
        }

        // set up and write the face elements
        plyFace face;
        int verts[256];
        face.verts = verts;
        vtkPLY::ply_put_element_setup(ply, "face");
        vtkIdType npts = 0;
        vtkIdType *pts = nullptr;
        for (polys->InitTraversal(), i = 0; i < numPolys; i++)
        {
            polys->GetNextCell(npts, pts);
            if (npts > 256)
            {
                vtkErrorMacro(<< "Ply file only supports polygons with <256 points");
            }
            else
            {
                for (j = 0; j < npts; j++)
                {
                    face.nverts = npts;
                    verts[j] = (int)pts[j];
                }
                if (cellColors)
                {
#ifdef RVLVTK7_1
                    idx = 3 * i;
                    face.red = *(cellColors + idx);
                    face.green = *(cellColors + idx + 1);
                    face.blue = *(cellColors + idx + 2);
#else
                    idx = cellAlpha ? 4 * i : 3 * i;
                    face.red = cellColors->GetValue(idx);
                    face.green = cellColors->GetValue(idx + 1);
                    face.blue = cellColors->GetValue(idx + 2);
                    if (cellAlpha)
                    {
                        face.alpha = cellColors->GetValue(idx + 3);
                    }
#endif
                }
                vtkPLY::ply_put_element(ply, (void *)&face);
            }
        } // for all polygons

        // close the PLY file
        vtkPLY::ply_close(ply);
    }

    vtkSmartPointer<vtkPolyData> DisplayIsoSurface(
        Array3D<float> f,
        float *P0,
        float voxelSize,
        float isolevel,
        float *W)
    {
        CRVLMem mem;

        mem.Create(5 * sizeof(QLIST::Entry<MCTriangle<float>>) + sizeof(BYTE *) + 1);

        QList<QLIST::Entry<MCTriangle<float>>> triangleList;

        QList<QLIST::Entry<MCTriangle<float>>> *pTriangleList = &triangleList;

        MarchingCubes MC;

        MC.ComputeTables();

        vtkSmartPointer<vtkPoints> points =
            vtkSmartPointer<vtkPoints>::New();

        vtkSmartPointer<vtkCellArray> triangles =
            vtkSmartPointer<vtkCellArray>::New();

        int nVoxels = f.a * f.b * f.c;

        int *iEdgePt[3];

        int i;

        for (i = 0; i < 3; i++)
        {
            iEdgePt[i] = new int[nVoxels];

            memset(iEdgePt[i], 0xff, nVoxels * sizeof(int));
        }

        int edgePtIdx[] = {-1, -1, -1, -1, -1, 1, 0, -1, -1, -1, 2, -1};
        int dx[12];
        int dy[12];
        int dz[12];
        int iEdge[12];

        dx[0] = 0;
        dy[0] = -1;
        dz[0] = -1;
        iEdge[0] = 0;
        dx[1] = 0;
        dy[1] = 0;
        dz[1] = -1;
        iEdge[1] = 1;
        dx[2] = 0;
        dy[2] = 0;
        dz[2] = -1;
        iEdge[2] = 0;
        dx[3] = -1;
        dy[3] = 0;
        dz[3] = -1;
        iEdge[3] = 1;
        dx[4] = 0;
        dy[4] = -1;
        dz[4] = 0;
        iEdge[4] = 0;
        dx[5] = 0;
        dy[5] = 0;
        dz[5] = 0;
        iEdge[5] = 1;
        dx[6] = 0;
        dy[6] = 0;
        dz[6] = 0;
        iEdge[6] = 0;
        dx[7] = -1;
        dy[7] = 0;
        dz[7] = 0;
        iEdge[7] = 1;
        dx[8] = -1;
        dy[8] = -1;
        dz[8] = 0;
        iEdge[8] = 2;
        dx[9] = 0;
        dy[9] = -1;
        dz[9] = 0;
        iEdge[9] = 2;
        dx[10] = 0;
        dy[10] = 0;
        dz[10] = 0;
        iEdge[10] = 2;
        dx[11] = -1;
        dy[11] = 0;
        dz[11] = 0;
        iEdge[11] = 2;

        int iTriangle = 0;

        int iPt = 0;

        int maxi = f.a - 2;
        int maxj = f.b - 2;
        int maxk = f.c - 2;

        float P[8][3];
        float F[8];
        QLIST::Entry<MCTriangle<float>> *pTriangle;
        vtkSmartPointer<vtkTriangle> triangle;
        int j, k, l;
        float x, y, z;
        int edgePtIdx_, edgeIdx;
        int iVoxel, iVoxel_, i_, j_, k_, iPt_;
        int jVoxel[8];

        for (k = 0; k <= maxk; k++)
            for (j = 0; j <= maxj; j++)
                for (i = 0; i <= maxi; i++)
                {
                    x = (float)i * voxelSize + P0[0];
                    y = (float)j * voxelSize + P0[1];
                    z = (float)k * voxelSize + P0[2];

                    iVoxel = RVL3DARRAY_INDEX(f, i, j, k);

                    RVLQLIST_INIT(pTriangleList);

                    jVoxel[0] = f.a * (f.b * k + j) + i;
                    jVoxel[1] = f.a * (f.b * k + j) + i + 1;
                    jVoxel[2] = f.a * (f.b * k + j + 1) + i + 1;
                    jVoxel[3] = f.a * (f.b * k + j + 1) + i;
                    jVoxel[4] = f.a * (f.b * (k + 1) + j) + i;
                    jVoxel[5] = f.a * (f.b * (k + 1) + j) + i + 1;
                    jVoxel[6] = f.a * (f.b * (k + 1) + j + 1) + i + 1;
                    jVoxel[7] = f.a * (f.b * (k + 1) + j + 1) + i;

                    RVLSET3VECTOR(P[0], x, y, z);
                    RVLSET3VECTOR(P[1], x + voxelSize, y, z);
                    RVLSET3VECTOR(P[2], x + voxelSize, y + voxelSize, z);
                    RVLSET3VECTOR(P[3], x, y + voxelSize, z);
                    RVLSET3VECTOR(P[4], x, y, z + voxelSize);
                    RVLSET3VECTOR(P[5], x + voxelSize, y, z + voxelSize);
                    RVLSET3VECTOR(P[6], x + voxelSize, y + voxelSize, z + voxelSize);
                    RVLSET3VECTOR(P[7], x, y + voxelSize, z + voxelSize);

                    F[0] = f.Element[jVoxel[0]];
                    F[1] = f.Element[jVoxel[1]];
                    F[2] = f.Element[jVoxel[2]];
                    F[3] = f.Element[jVoxel[3]];
                    F[4] = f.Element[jVoxel[4]];
                    F[5] = f.Element[jVoxel[5]];
                    F[6] = f.Element[jVoxel[6]];
                    F[7] = f.Element[jVoxel[7]];

                    MC.ComputeTriangles<float>(P, F, isolevel, pTriangleList, &mem);

                    pTriangle = pTriangleList->pFirst;

                    while (pTriangle)
                    {
                        for (l = 0; l < 3; l++)
                        {
                            edgePtIdx_ = edgePtIdx[pTriangle->data.idx[l]];

                            if (edgePtIdx_ < 0)
                                continue;

                            if (iEdgePt[edgePtIdx_][iVoxel] >= 0)
                                continue;

                            points->InsertNextPoint(pTriangle->data.P[l][0], pTriangle->data.P[l][1], pTriangle->data.P[l][2]);

                            iEdgePt[edgePtIdx_][iVoxel] = iPt;

                            iPt++;
                        }

                        pTriangle = pTriangle->pNext;
                    }

                    if (W)
                    {
                        for (l = 0; l < 8; l++)
                            if (W[jVoxel[l]] < 0.1f)
                                break;

                        if (l < 8)
                        {
                            mem.Clear();

                            continue;
                        }
                    }

                    pTriangle = pTriangleList->pFirst;

                    while (pTriangle)
                    {
                        triangle = vtkSmartPointer<vtkTriangle>::New();

                        for (l = 0; l < 3; l++)
                        {
                            edgeIdx = pTriangle->data.idx[l];

                            i_ = i + dx[edgeIdx];

                            if (i_ < 0)
                                break;

                            if (i_ > maxi)
                                break;

                            j_ = j + dy[edgeIdx];

                            if (j_ < 0)
                                break;

                            if (j_ > maxj)
                                break;

                            k_ = k + dz[edgeIdx];

                            if (k_ < 0)
                                break;

                            if (k_ > maxk)
                                break;
                        }

                        if (l >= 3)
                        {
                            for (l = 0; l < 3; l++)
                            {
                                edgeIdx = pTriangle->data.idx[l];

                                i_ = i + dx[edgeIdx];
                                j_ = j + dy[edgeIdx];
                                k_ = k + dz[edgeIdx];

                                iVoxel_ = RVL3DARRAY_INDEX(f, i_, j_, k_);

                                iPt_ = iEdgePt[iEdge[edgeIdx]][iVoxel_];

                                triangle->GetPointIds()->SetId(l, iPt_);
                            }

                            // points->InsertNextPoint(pTriangle->data.P[0][0], pTriangle->data.P[0][1], pTriangle->data.P[0][2]);
                            // points->InsertNextPoint(pTriangle->data.P[1][0], pTriangle->data.P[1][1], pTriangle->data.P[1][2]);
                            // points->InsertNextPoint(pTriangle->data.P[2][0], pTriangle->data.P[2][1], pTriangle->data.P[2][2]);

                            // triangle = vtkSmartPointer<vtkTriangle>::New();
                            // triangle->GetPointIds()->SetId(0, 3 * iTriangle);
                            // triangle->GetPointIds()->SetId(1, 3 * iTriangle + 1);
                            // triangle->GetPointIds()->SetId(2, 3 * iTriangle + 2);

                            triangles->InsertNextCell(triangle);

                            iTriangle++;
                        }

                        pTriangle = pTriangle->pNext;
                    } // for every triangle in pTriangleList

                    mem.Clear();
                } // for every voxel

        printf("no. of pts. = %d\n", points->GetNumberOfPoints());

        printf("no. of triangles = %d\n", triangles->GetNumberOfCells());

        // Create a polydata object
        vtkSmartPointer<vtkPolyData> polyData =
            vtkSmartPointer<vtkPolyData>::New();

        // Add the geometry and topology to the polydata
        polyData->SetPoints(points);
        polyData->SetPolys(triangles);

        delete[] iEdgePt[0];
        delete[] iEdgePt[1];
        delete[] iEdgePt[2];

        return polyData;
    }
#endif

    void RandomColors(
        unsigned char *SelectionColor,
        unsigned char *&colorArray,
        int n)
    {
        int SelectionColor_[3];

        RVLCONVTOINT3(SelectionColor, SelectionColor_);

        colorArray = new unsigned char[3 * n];

        int iNode;
        int Color[3], dColor[3];
        unsigned char *NodeColor_;

        for (iNode = 0; iNode < n; iNode++)
        {
            do
            {
                Color[0] = rand() % 256;
                Color[1] = rand() % 256;
                Color[2] = rand() % 256;

                RVLDIF3VECTORS(Color, SelectionColor, dColor);
            } while (RVLDOTPRODUCT3(dColor, dColor) < 128 * 128);

            NodeColor_ = colorArray + 3 * iNode;

            NodeColor_[0] = (unsigned char)Color[0];
            NodeColor_[1] = (unsigned char)Color[1];
            NodeColor_[2] = (unsigned char)Color[2];
        }
    }

    // Remove the identical function from PlanarSurfelDetector class.

    void RandomIndices(Array<int> &A)
    {
#ifdef RVLPLANARSURFELDETECTOR_PSEUDO_RANDOM_DEBUG
        FILE *fp = fopen("..\\pseudorandom1000000.dat", "rb");

        int *iRnd = new int[A.n];

        // for (int i = 0; i < 1000000; i++)
        //	iRnd[i] = (rand() % 0x100) + (rand() % 0x100) * 0x100 + (rand() % 0x100) * 0x10000 + (rand() % 0x80) * 0x1000000;

        // fwrite(iRnd, sizeof(int), 1000000, fp);

        fread(iRnd, sizeof(int), A.n, fp);

        fclose(fp);

        int *piRnd = iRnd;
#endif

        A.Element = new int[A.n];

        int iPt;

        for (iPt = 0; iPt < A.n; iPt++)
            A.Element[iPt] = iPt;

        int iPt_;
        int iTmp;

        // srand(time(NULL)); //VIDOVIC RANDOM TEST

        for (iPt = 0; iPt < A.n; iPt++)
        {
#ifdef RVLPLANARSURFELDETECTOR_PSEUDO_RANDOM_DEBUG
            iPt_ = (*(piRnd++)) % A.n;
#else
            iPt_ = rand() % A.n;
#endif

            iTmp = A.Element[iPt];
            A.Element[iPt] = A.Element[iPt_];
            A.Element[iPt_] = iTmp;
        }

#ifdef RVLPLANARSURFELDETECTOR_PSEUDO_RANDOM_DEBUG
        delete[] iRnd;
#endif
    }

    void DistributionFromCovMx(
        float *C,
        float *R,
        float *var)
    {
        Eigen::EigenSolver<Eigen::Matrix3f> eigenSolver;

        eigenSolver.compute(Eigen::Map<Eigen::Matrix3f>(C));

        Eigen::Matrix3f R_ = eigenSolver.pseudoEigenvectors();

        R[0] = R_(0, 0);
        R[1] = R_(1, 0);
        R[2] = R_(2, 0);
        R[3] = R_(0, 1);
        R[4] = R_(1, 1);
        R[5] = R_(2, 1);

        float *X = R;
        float *Y = R + 3;
        float *Z = R + 6;

        RVLCROSSPRODUCT3(X, Y, Z);

        Eigen::Vector3cf var_ = eigenSolver.eigenvalues();

        var[0] = var_[0].real();
        var[1] = var_[1].real();
        var[2] = var_[2].real();
    }

    char *ReadConfigurationFile(char *cfgSelectionFileName)
    {
        char line[500];

        FILE *fp = fopen(cfgSelectionFileName, "r");

        if (fp == NULL)
        {
            printf("ERROR: Cannot load configuration file %s!\n", cfgSelectionFileName);
            return NULL;
        }

        char *cfgFileName = NULL;
        int linelen;

        while (!feof(fp))
        {
            fgets(line, 500, fp);

            if (line[0] == '%')
                continue;

            linelen = strlen(line);

            if (line[linelen - 1] == 10)
                line[linelen - 1] = 0;

            cfgFileName = RVLCreateString(line);
        }

        fclose(fp);

        return cfgFileName;
    }

    void MeanShift1D(
        Array<int> histogram,
        int winSize,
        int minClusterSize,
        Array<int> &clusters,
        int *assignment,
        int *wAgg,
        int *move,
        int *w)
    {
        int halfWinSize = (winSize - 1) / 2;

        if (clusters.Element == NULL)
            clusters.Element = new int[histogram.n];

        if (assignment == NULL)
            assignment = new int[histogram.n];

        if (wAgg == NULL)
            wAgg = new int[histogram.n];

        bool bAllocateMove = (move == NULL);

        if (bAllocateMove)
            move = new int[histogram.n];

        bool bAllocatew = (w == NULL);

        if (bAllocatew)
            w = new int[histogram.n];

        int i, j, k, move_, w_;

        for (i = 0; i < histogram.n; i++)
        {
            w_ = 0;
            move_ = 0;

            for (j = -halfWinSize; j <= halfWinSize; j++)
            {
                k = ((i + j + histogram.n) % histogram.n);

                move_ += ((j + halfWinSize) * histogram.Element[k]);

                w_ += histogram.Element[k];
            }

            move_ = (w_ > 0 ? (2 * move_ + w_) / (2 * w_) - halfWinSize : 0);

            move[i] = move_;

            w[i] = w_;
        }

        memset(wAgg, 0, histogram.n * sizeof(int));

        int sign;

        for (i = 0; i < histogram.n; i++)
        {
            if (w[i] == 0)
            {
                assignment[i] = -1;

                continue;
            }

            sign = (move[i] >= 0 ? 1 : -1);

            j = i;

            while (sign * move[j] > 0)
                j = (j + move[j] + histogram.n) % histogram.n;

            if (move[j] < 0)
                j = (j + move[j] + histogram.n) % histogram.n;

            wAgg[j] += histogram.Element[i];

            assignment[i] = j;
        }

        clusters.n = 0;

        for (i = 0; i < histogram.n; i++)
            if (wAgg[i] >= minClusterSize)
                clusters.Element[clusters.n++] = i;

        if (bAllocateMove)
            delete[] move;

        if (bAllocatew)
            delete[] w;
    }

    void CreateImage3x3NeighborhoodLT(
        int w,
        int *neighbor)
    {
        neighbor[0] = -1 - w;
        neighbor[1] = -w;
        neighbor[2] = 1 - w;
        neighbor[3] = -1;
        neighbor[4] = 0;
        neighbor[5] = 1;
        neighbor[6] = -1 + w;
        neighbor[7] = w;
        neighbor[8] = 1 + w;
    }

    void CreateGrayScaleImage(Array2D<float> inputImage, cv::Mat &outputImage)
    {
        cv::Mat inputImage_(inputImage.h, inputImage.w, CV_32FC1, inputImage.Element);

        double minI, maxI;

        cv::minMaxLoc(inputImage_, &minI, &maxI);

        float range = (float)(maxI - minI);

        inputImage_.convertTo(outputImage, CV_8U, 255.0 / range, -255.0 * minI / range);
    }

    void SetCameraParams(Camera &camera, float fu, float fv, float uc, float vc, int w, int h)
    {
        camera.fu = fu;
        camera.fv = fv;
        camera.uc = uc;
        camera.vc = vc;
        camera.w = w;
        camera.h = h;
    }

    // Vidovic
    void UnionOfIndices(
        Array<int> &iInArray1,
        Array<int> &iInArray2,
        Array<int> &iOutArray,
        bool *bIndicesInArray)
    {
        iOutArray.n = 0;
        int iVertex, i;

        // copy from iInArray1
        for (i = 0; i < iInArray1.n; i++)
        {
            iVertex = iInArray1.Element[i];

            if (bIndicesInArray[iVertex])
                continue;

            iOutArray.Element[iOutArray.n++] = iVertex;

            bIndicesInArray[iVertex] = true;
        }

        // copy from iInArray2
        for (i = 0; i < iInArray2.n; i++)
        {
            iVertex = iInArray2.Element[i];

            if (bIndicesInArray[iVertex])
                continue;

            iOutArray.Element[iOutArray.n++] = iVertex;

            bIndicesInArray[iVertex] = true;
        }

        for (i = 0; i < iOutArray.n; i++)
            bIndicesInArray[iOutArray.Element[i]] = false;
    }

    bool CheckFlag(unsigned char flags, unsigned char flagToCheck)
    {
        return flags & flagToCheck;
    }

    bool CheckFlags(unsigned char flags, unsigned char flagsToCheck)
    {
        return flags == flagsToCheck;
    }
    // End Vidovic
} // namespace RVL

void FilterImage(cv::Mat img)
{
    cv::Mat newImg(480, 640, CV_16UC1, cv::Scalar::all(0));
    img.copyTo(newImg);
    float sum = 0;
    float max = 0;
    int no = 0;
    for (int y = 10; y < (img.rows - 10); y++)
    {
        for (int x = 10; x < (img.cols - 10); x++)
        {
            if (img.at<uint16_t>(y, x) > 0)
                continue;
            // inner
            sum = 0;
            no = 0;
            max = 0;
            for (int v = -1; v < 1; v++)
            {
                for (int u = -1; u < 1; u++)
                {
                    if (img.at<uint16_t>(y + v, x + u) == 0)
                        continue;
                    sum += img.at<uint16_t>(y + v, x + u);
                    no++;
                    if (img.at<uint16_t>(y + v, x + u) > max)
                        max = img.at<uint16_t>(y + v, x + u);
                }
            }
            if (no != 0)
                newImg.at<uint16_t>(y, x) = max; // sum / no;
        }
    }
    newImg.copyTo(img);
}
