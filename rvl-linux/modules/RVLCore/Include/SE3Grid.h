#pragma once

namespace RVL
{
    struct SE3Point
    {
        Pose3D pose;
        int idx;
        bool bFree;
        int iNext;
    };

    class SE3Grid
    {
    public:
        SE3Grid();
        virtual ~SE3Grid();
        void Create(
            Box<float> volumeIn,
            float posCellSizeIn = 0.02,
            int nZCellsIn = 66,
            int nRollCellsIn = 16,
            int expectedNumberOfNodesIn = 100000);
        void Clear();
        bool Add(
            Pose3D point,
            bool bFree = true,
            int idx = -1,
            int iPosCellIn = -1,
            int iZIn = -1,
            int iRollIn = -1);
        int Fetch(
            Pose3D point,
            int &iPosCell,
            int &iZ,
            int &iRoll);
        void Neighbors(
            Pose3D point,
            float rPos,
            float rOrient,
            Array<int> &neighbors);
        bool Cell(
            Pose3D point,
            int &iPosX,
            int &iPoxY,
            int &iPosZ,
            int &iPosCell,
            int &iZ,
            int &iRoll);
        inline void PosCell(
            float *t,
            int &i,
            int &j,
            int &k)
        {
            i = (int)floor((t[0] - volume.minx) / posCellSize);
            j = (int)floor((t[1] - volume.miny) / posCellSize);
            k = (int)floor((t[2] - volume.minz) / posCellSize);
        };
        inline int PosCellIdx(
            int i,
            int j,
            int k)
        {
            return (i >= 0 && i < grid.a && j >= 0 && j < grid.b && k >= 0 && k < grid.c ? (k * grid.b + j) * grid.a + i : -1);
        }

    public:
        Array3D<int> grid;
        int expectedNumberOfNodes;
        Box<float> volume;
        Array<SE3Point> points;
        float posCellSize;
        float ZCellSize;
        float rollCellSize;
        Array2D<float> A;
        float *XMem;
        bool bSingleNodePerCell;

    private:
        int *zMem;
        int *rollMem;
        int nZGrids;
        int nRollArrays;
        int nPosCells;
        int nZCells;
        int nRollCells;
        ConvexTemplateLookUpTable CTLUT;
        int zMemSize;
        int rollMemSize;
        int pointMemSize;
        Box<int> volumeIdxMargins;
        Array<int> *neighborZCells;
        Array<int> allZCells;
        int *neighborZCellMem;
        int *neighborMem;
        int neighborMemCapacity;
    };
} // namespace RVL
