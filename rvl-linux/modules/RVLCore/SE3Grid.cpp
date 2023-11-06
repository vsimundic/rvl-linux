#include "RVLCore2.h"
#include "RVLVTK.h"
#include "Util.h"
#include "SE3Grid.h"

using namespace RVL;

SE3Grid::SE3Grid()
{
    posCellSize = 0.02; // m
    nZCells = 66;
    nRollCells = 16;
    grid.Element = NULL;
    zMem = NULL;
    rollMem = NULL;
    points.Element = NULL;
    A.Element = NULL;
    XMem = NULL;
    CTLUT.LUT.Element = NULL;
    neighborZCells = NULL;
    neighborZCellMem = NULL;
    allZCells.Element = NULL;
    bSingleNodePerCell = true; // Currently, this is the only option. In the future, an option with multiple nodes per cell will be allowed.
    neighborMem = NULL;
}

SE3Grid::~SE3Grid()
{
    Clear();
}

void SE3Grid::Create(
    Box<float> volumeIn,
    float posCellSizeIn,
    int nZCellsIn,
    int nRollCellsIn,
    int expectedNumberOfNodesIn)
{
    // Sample unit sphere.

    if (nZCellsIn == 66)
    {
        A.w = 3;
        A.h = 66;
        RVL_DELETE_ARRAY(A.Element);
        A.Element = new float[A.w * A.h];
        CreateConvexTemplate66(A.Element);
    }
    else
    {
        printf("ERROR: In this version of RVL, the only number of cells in z-grid allowed is 66!\n");

        return;
    }
    CreateTemplateLookUpTable(A, 200, CTLUT);

    // For each unit sphere sample, compute an axis perpendicular to the given sample.

    RVL_DELETE_ARRAY(XMem);
    XMem = new float[3 * A.h];
    int iZ;
    int i, j, k;
    float *X, *Z;
    float fTmp;
    for (iZ = 0; iZ < A.h; iZ++)
    {
        Z = A.Element + 3 * iZ;
        X = XMem + 3 * iZ;
        RVLORTHOGONAL3(Z, X, i, j, k, fTmp);
    }

    // Set parameters.

    float zNeighborhoodSizeDeg = 45.0f; // deg

    nZCells = nZCellsIn;
    volume = volumeIn;
    nRollCells = nRollCellsIn;
    posCellSize = posCellSizeIn;
    expectedNumberOfNodes = expectedNumberOfNodesIn;
    rollCellSize = 2 * PI / (float)nRollCells;

    // Constants.

    float csZNeighborhoodSize = cos(DEG2RAD * zNeighborhoodSizeDeg * 1.1f);

    // Allocate and initialize position grid.

    float volumeSize[3];
    BoxSize<float>(&volume, volumeSize[0], volumeSize[1], volumeSize[2]);
    grid.a = (int)floor(volumeSize[0] / posCellSize) + 1;
    grid.b = (int)floor(volumeSize[1] / posCellSize) + 1;
    grid.c = (int)floor(volumeSize[2] / posCellSize) + 1;
    nPosCells = grid.a * grid.b * grid.c;
    RVL_DELETE_ARRAY(grid.Element);
    grid.Element = new int[nPosCells];
    memset(grid.Element, 0xff, nPosCells * sizeof(int));

    // Allocate and initialize z-grid memory storage.

    RVL_DELETE_ARRAY(zMem);
    zMemSize = expectedNumberOfNodes;
    zMem = new int[nZCells * zMemSize];
    memset(zMem, 0xff, nZCells * zMemSize * sizeof(int));
    nZGrids = 0;

    // Allocate and initialize roll array memory storage.

    RVL_DELETE_ARRAY(rollMem)
    rollMemSize = expectedNumberOfNodes;
    rollMem = new int[nRollCells * rollMemSize];
    memset(rollMem, 0xff, nRollCells * rollMemSize * sizeof(int));
    nRollArrays = 0;

    // Allocate and initialize SE(3) point memory storage.

    RVL_DELETE_ARRAY(points.Element);
    pointMemSize = expectedNumberOfNodes;
    points.Element = new SE3Point[expectedNumberOfNodes];
    points.n = 0;

    // Volume index margins.

    volumeIdxMargins.minx = 0;
    volumeIdxMargins.maxx = grid.a - 1;
    volumeIdxMargins.miny = 0;
    volumeIdxMargins.maxy = grid.b - 1;
    volumeIdxMargins.minz = 0;
    volumeIdxMargins.maxz = grid.c - 1;

    // Neighbor z-cells.

    RVL_DELETE_ARRAY(neighborZCells);
    neighborZCells = new Array<int>[A.h];
    std::vector<int> neighborZCells_;
    float *Z_;
    int iZ_;
    int iFirstNeighborZCell = 0;
    for (iZ = 0; iZ < A.h; iZ++)
    {
        Z = A.Element + 3 * iZ;
        for (iZ_ = 0; iZ_ < A.h; iZ_++)
        {
            Z_ = A.Element + 3 * iZ_;
            if (RVLDOTPRODUCT3(Z, Z_) >= csZNeighborhoodSize)
                neighborZCells_.push_back(iZ_);
        }
        neighborZCells[iZ].n = neighborZCells_.size() - iFirstNeighborZCell;
        iFirstNeighborZCell = neighborZCells_.size();
    }
    RVL_DELETE_ARRAY(neighborZCellMem);
    neighborZCellMem = new int[neighborZCells_.size()];
    int *pZCellIdx = neighborZCellMem;
    j = 0;
    RVL_DELETE_ARRAY(allZCells.Element);
    allZCells.n = A.h;
    allZCells.Element = new int[allZCells.n];
    for (iZ = 0; iZ < A.h; iZ++)
    {
        neighborZCells[iZ].Element = pZCellIdx;
        for (i = 0; i < neighborZCells[iZ].n; i++)
            neighborZCells[iZ].Element[i] = neighborZCells_[j++];
        pZCellIdx += neighborZCells[iZ].n;
        allZCells.Element[iZ] = iZ;
    }

    // Neighbor memory.

    RVL_DELETE_ARRAY(neighborMem);
    neighborMemCapacity = 10000;
    neighborMem = new int[neighborMemCapacity];
}

void SE3Grid::Clear()
{
    RVL_DELETE_ARRAY(grid.Element);
    RVL_DELETE_ARRAY(zMem);
    RVL_DELETE_ARRAY(rollMem);
    RVL_DELETE_ARRAY(points.Element);
    RVL_DELETE_ARRAY(A.Element);
    RVL_DELETE_ARRAY(XMem);
    RVL_DELETE_ARRAY(CTLUT.LUT.Element);
    RVL_DELETE_ARRAY(neighborZCells);
    RVL_DELETE_ARRAY(neighborZCellMem);
    RVL_DELETE_ARRAY(allZCells.Element);
    RVL_DELETE_ARRAY(neighborMem);
}

bool SE3Grid::Add(
    Pose3D point,
    bool bFree,
    int idx,
    int iPosCellIn,
    int iZIn,
    int iRollIn)
{
    // Get cell.

    int iPosCell, iZ, iRoll;
    if (iPosCellIn < 0)
    {
        int i, j, k;
        if (!Cell(point, i, j, k, iPosCell, iZ, iRoll))
            return false;
    }
    else
    {
        iPosCell = iPosCellIn;
        iZ = iZIn;
        iRoll = iRollIn;
    }

    // Get Z-grid.

    int iZGrid = grid.Element[iPosCell];
    int *zGrid;
    int iRollArray;
    if (iZGrid < 0)
    {
        if (nZGrids >= zMemSize)
        {
            int *pTmp = zMem;
            int newZMemSize = 2 * zMemSize;
            zMem = new int[nZCells * newZMemSize];
            memset(grid.Element, 0xff, nZCells * newZMemSize * sizeof(int));
            memcpy(zMem, pTmp, nZCells * zMemSize * sizeof(int));
            zMemSize = newZMemSize;
            delete[] pTmp;
        }
        iZGrid = nZGrids;
        grid.Element[iPosCell] = iZGrid;
        nZGrids++;
    }
    zGrid = zMem + nZCells * iZGrid;

    // Get Roll array.

    iRollArray = zGrid[iZ];
    int *rollArray;
    if (iRollArray < 0)
    {
        if (nRollArrays >= rollMemSize)
        {
            int *pTmp = rollMem;
            int newRollMemSize = 2 * rollMemSize;
            rollMem = new int[nRollCells * newRollMemSize];
            memset(grid.Element, 0xff, nRollCells * newRollMemSize * sizeof(int));
            memcpy(rollMem, pTmp, nRollCells * rollMemSize * sizeof(int));
            rollMemSize = newRollMemSize;
            delete[] pTmp;
        }
        iRollArray = nRollArrays;
        zGrid[iZ] = iRollArray;
        nRollArrays++;
    }
    rollArray = rollMem + nRollCells * iRollArray;

    // If the cell is already occupied, don't insert the new point.

    if (rollArray[iRoll] >= 0)
        return false;

    // Insert the new point.

    if (points.n >= pointMemSize)
    {
        SE3Point *pTmp = points.Element;
        int newPointMemSize = 2 * pointMemSize;
        points.Element = new SE3Point[newPointMemSize];
        memcpy(points.Element, pTmp, pointMemSize * sizeof(SE3Point));
        pointMemSize = newPointMemSize;
        delete[] pTmp;
    }
    SE3Point newPoint;
    newPoint.pose = point;
    newPoint.bFree = bFree;
    newPoint.idx = idx;
    points.Element[points.n] = newPoint;
    rollArray[iRoll] = points.n;
    points.n++;

    return true;
}

int SE3Grid::Fetch(
    Pose3D point,
    int &iPosCell,
    int &iZ,
    int &iRoll)
{
    int i, j, k;
    if (!Cell(point, i, j, k, iPosCell, iZ, iRoll))
        return -1;
    int iZGrid = grid.Element[iPosCell];
    if (iZGrid < 0)
        return -1;
    int *zGrid = zMem + nZCells * iZGrid;
    int iRollArray = zGrid[iZ];
    if (iRollArray < 0)
        return -1;
    int *rollArray = rollMem + iRollArray * nRollCells;
    int iPoint = rollArray[iRoll];
    if (iPoint < 0)
        return -1;
    return iPoint;
}

void SE3Grid::Neighbors(
    Pose3D point,
    float rPos,
    float rOrient,
    Array<int> &neighbors)
{
    bool bOrient = (rOrient > 0.0f && rOrient < PI);
    neighbors.n = 0;
    neighbors.Element = neighborMem;
    Box<int> ROI;
    float V3Tmp[3];
    RVLSET3VECTOR(V3Tmp, rPos, rPos, rPos);
    Pose3D poseMinVertex;
    RVLCOPYMX3X3(point.R, poseMinVertex.R);
    RVLDIF3VECTORS(point.t, V3Tmp, poseMinVertex.t);
    int iPosCell, iZ, iRoll;
    Cell(poseMinVertex, ROI.minx, ROI.miny, ROI.minz, iPosCell, iZ, iRoll);
    float ROIMaxVertex[3];
    RVLSUM3VECTORS(point.t, V3Tmp, ROIMaxVertex);
    PosCell(ROIMaxVertex, ROI.maxx, ROI.maxy, ROI.maxz);
    Box<int> ROICropped;
    if (!BoxIntersection<int>(&ROI, &volumeIdxMargins, &ROICropped))
        return;
    int i, j, k, l, m;
    int iZGrid;
    int *zGrid;
    int iRollArray;
    int *rollArray;
    Array<int> *pNeighborZCells;
    int iZ_, iPoint;
    int rollNeighborhood;
    float rPos2 = rPos * rPos;
    float csrOrient;
    if (bOrient)
    {
        rollNeighborhood = (int)ceil(rOrient / rollCellSize) + 1;
        csrOrient = cos(rOrient);
    }
    neighbors.n = 0;
    SE3Point *pNeighbor;
    float dist2, cs;
    float dR[9];
    int iRollFirst, iRollLast;
    for (k = ROICropped.minz; k <= ROICropped.maxz; k++)
        for (j = ROICropped.miny; j <= ROICropped.maxy; j++)
            for (i = ROICropped.minx; i <= ROICropped.maxx; i++)
            {
                iPosCell = PosCellIdx(i, j, k);
                iZGrid = grid.Element[iPosCell];
                if (iZGrid >= 0)
                {
                    zGrid = zMem + nZCells * iZGrid;
                    pNeighborZCells = (bOrient ? neighborZCells + iZ : &allZCells);
                    for (l = 0; l < pNeighborZCells->n; l++)
                    {
                        iZ_ = pNeighborZCells->Element[l];
                        iRollArray = zGrid[iZ_];
                        if (iRollArray >= 0)
                        {
                            rollArray = rollMem + iRollArray * nRollCells;
                            if (bOrient)
                            {
                                iRollFirst = iRoll - rollNeighborhood;
                                iRollLast = iRoll + rollNeighborhood;
                            }
                            else
                            {
                                iRollFirst = 0;
                                iRollLast = nRollCells - 1;
                            }
                            for (m = iRollFirst; m <= iRollLast; m++)
                            {
                                iPoint = rollArray[m];
                                if (iPoint >= 0)
                                {
                                    pNeighbor = points.Element + iPoint;
                                    RVLDIF3VECTORS(pNeighbor->pose.t, point.t, V3Tmp);
                                    dist2 = RVLDOTPRODUCT3(V3Tmp, V3Tmp);
                                    if (dist2 <= rPos2)
                                    {
                                        if (bOrient)
                                        {
                                            RVLMXMUL3X3T1(pNeighbor->pose.R, point.R, dR);
                                            cs = RVLROTDIFF(dR);
                                            if (cs >= csrOrient)
                                                neighbors.Element[neighbors.n++] = iPoint;
                                        }
                                        else
                                            neighbors.Element[neighbors.n++] = iPoint;
                                        if (neighbors.n >= neighborMemCapacity)
                                        {
                                            neighborMemCapacity *= 2;
                                            neighborMem = new int[neighborMemCapacity];
                                            memcpy(neighborMem, neighbors.Element, neighbors.n * sizeof(int));
                                            delete[] neighbors.Element;
                                            neighbors.Element = neighborMem;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
}

bool SE3Grid::Cell(
    Pose3D point,
    int &iPosX,
    int &iPoxY,
    int &iPosZ,
    int &iPosCell,
    int &iZ,
    int &iRoll)
{
    // Position cell.

    PosCell(point.t, iPosX, iPoxY, iPosZ);
    iPosCell = PosCellIdx(iPosX, iPoxY, iPosZ);
    if (iPosCell < 0)
        return false;

    // Z-cell.

    float ZP0[3];
    RVLCOPYCOLMX3X3(point.R, 2, ZP0);
    float V3Tmp[3];
    float *I = NULL;
    int i, j, k;
    RVLGET_CLOSEST_CONVEX_TEMPLATE_ELEMENT(CTLUT, ZP0, I, V3Tmp, i, j, k, iZ);

    // Roll cell.

    float *ZG0 = A.Element + 3 * iZ;
    float U[3];
    RVLCROSSPRODUCT3(ZP0, ZG0, U);
    float th = sqrt(RVLDOTPRODUCT3(U, U));
    float roll;
    if (th >= 1e-6)
    {
        RVLSCALE3VECTOR2(U, th, U);
        float RAllign[9];
        AngleAxisToRot<float>(U, th, RAllign);
        float RPGAligned[9];
        RVLCOMPROT3D3DOF(RAllign, point.R, RPGAligned);
        roll = atan2(RPGAligned[3], RPGAligned[0]);
    }
    else
        roll = 0.0f;
    iRoll = (int)float((roll + PI) / rollCellSize);

    return true;
}