namespace RVL
{
	template<typename DataType, typename CoordinateType>
	class Space3DGrid
	{
	public:
		Space3DGrid()
		{
			cellMemSize = 0;
			dataMemSize = 0;

			grid.Element = NULL;
			dataMem = NULL;
			dataBuff = NULL;
			activeCellArray.Element = NULL;
			bActiveCell = NULL;

			neighborCellArray.Element = new int[8];
		}

		virtual ~Space3DGrid()
		{
			RVL_DELETE_ARRAY(grid.Element);
			RVL_DELETE_ARRAY(dataMem);
			RVL_DELETE_ARRAY(dataBuff);
			RVL_DELETE_ARRAY(activeCellArray.Element);
			RVL_DELETE_ARRAY(bActiveCell);

			delete[] neighborCellArray.Element;
		}

		void Create(
			int a,
			int b,
			int c,
			CoordinateType cellSize_,
			int maxnData)
		{
			grid.a = a;
			grid.b = b;
			grid.c = c;
			
			nCells = a * b * c + 1;

			if (nCells > cellMemSize)
			{
				RVL_DELETE_ARRAY(grid.Element);

				grid.Element = new QList<DataType>[nCells];

				cellMemSize = nCells;

				RVL_DELETE_ARRAY(activeCellArray.Element);

				activeCellArray.Element = new int[nCells];

				activeCellArray.n = 0;

				RVL_DELETE_ARRAY(bActiveCell);

				bActiveCell = new bool[nCells];
			}

			memset(bActiveCell, 0, nCells * sizeof(bool));

			iOutCell = nCells - 1;

			QList<DataType> *pCellDataList;

			int iCell;

			for (iCell = 0; iCell < nCells; iCell++)
			{
				pCellDataList = grid.Element + iCell;

				RVLQLIST_INIT(pCellDataList);
			}

			cellSize = cellSize_;

			halfCellSize = 0.5 * cellSize;

			cellSize2 = cellSize * cellSize;

			if (maxnData > dataMemSize)
			{
				dataMemSize = maxnData;

				RVL_DELETE_ARRAY(dataMem);

				dataMem = new DataType[dataMemSize];

				RVL_DELETE_ARRAY(dataBuff);

				dataBuff = new DataType *[dataMemSize];
			}

			pNewData = dataMem;
		}

		void SetVolume(
			CoordinateType minx,
			CoordinateType miny,
			CoordinateType minz)
		{
			volume.minx = minx;
			volume.maxx = minx + cellSize * (CoordinateType)(grid.a);
			volume.miny = miny;
			volume.maxy = miny + cellSize * (CoordinateType)(grid.b);
			volume.minz = minz;
			volume.maxz = minz + cellSize * (CoordinateType)(grid.c);
		}

		inline void Cell(
			CoordinateType *P,
			int &i,
			int &j,
			int &k)
		{
			i = (int)floor((P[0] - volume.minx) / cellSize);
			j = (int)floor((P[1] - volume.miny) / cellSize);
			k = (int)floor((P[2] - volume.minz) / cellSize);
		}

		inline int Cell(
			int i, 
			int j, 
			int k)
		{
			return (i >= 0 && i < grid.a && j >= 0 && j < grid.b && k >= 0 && k < grid.c ? (k * grid.b + j) * grid.a + i : iOutCell);
		}

		inline void AddData(DataType &data)
		{
			*pNewData = data;

			int i, j, k;

			Cell(data.P, i, j, k);

			int iCell = Cell(i, j, k);

			QList<DataType> *pCellDataList = grid.Element + iCell;

			pNewData->iCell = iCell;

			if (!bActiveCell[iCell])
			{
				activeCellArray.Element[activeCellArray.n++] = iCell;

				bActiveCell[iCell] = true;
			}				

			RVLQLIST_ADD_ENTRY2(pCellDataList, pNewData);

			pNewData++;
		}

		inline void RemoveData(DataType *pData)
		{
			QList<DataType> *pCellDataList = grid.Element + pData->iCell;

			RVLQLIST_REMOVE_ENTRY2(pCellDataList, pData, DataType);
		}

		void Neighbors(
			CoordinateType *P,
			Array<DataType *> &neighborArray)
		{
			CoordinateType P_[3];

			P_[0] = P[0] - halfCellSize;
			P_[1] = P[1] - halfCellSize;
			P_[2] = P[2] - halfCellSize;

			int i, j, k;

			Cell(P_, i, j, k);

			int iCell = Cell(i, j, k);

			bool bOutCell = (iCell == iOutCell);

			neighborCellArray.n = 0;

			int i_, j_, k_;

			for (i_ = i; i_ <= i + 1; i_++)
			{
				if (i_ >= 0 && i_ < grid.a)
				{
					for (j_ = j; j_ <= j + 1; j_++)
					{
						if (j_ >= 0 && j_ < grid.b)
						{
							for (k_ = k; k_ <= k + 1; k_++)
							{
								if (k_ >= 0 && k_ < grid.c)
									neighborCellArray.Element[neighborCellArray.n++] = (k_ * grid.b + j_) * grid.a + i_;
								else
									bOutCell = true;
							}
						}
						else
							bOutCell = true;
					}
				}
				else
					bOutCell = true;
			}	

			if (bOutCell)
				neighborCellArray.Element[neighborCellArray.n++] = iOutCell;

			QList<DataType> *pCellDataList;

			neighborArray.n = 0;

			neighborArray.Element = dataBuff;

			DataType *pData;
			CoordinateType dist2;

			for (i = 0; i < neighborCellArray.n; i++)
			{
				pCellDataList = grid.Element + neighborCellArray.Element[i];

				pData = pCellDataList->pFirst;

				while (pData)
				{
					RVLDIF3VECTORS(pData->P, P, P_);

					dist2 = RVLDOTPRODUCT3(P_, P_);

					if (dist2 <= cellSize2)
						neighborArray.Element[neighborArray.n++] = pData;

					pData = pData->pNext;
				}
			}
		}

		void GetData(Array<DataType *> &dataArray)
		{
			dataArray.n = 0;

			dataArray.Element = dataBuff;

			int i;
			DataType *pData;
			QList<DataType> *pCellDataList;

			for (i = 0; i < activeCellArray.n; i++)
			{
				pCellDataList = grid.Element + activeCellArray.Element[i];

				pData = pCellDataList->pFirst;

				while (pData)
				{
					if (pData->iMatch == 174)
						int debug = 0;

					dataArray.Element[dataArray.n++] = pData;

					pData = pData->pNext;
				}
			}
		}

		void Clear()
		{
			int i, iCell;
			QList<DataType> *pCellDataList;

			for (i = 0; i < activeCellArray.n; i++)
			{
				iCell = activeCellArray.Element[i];

				pCellDataList = grid.Element + iCell;

				RVLQLIST_INIT(pCellDataList);

				bActiveCell[iCell] = false;
			}

			activeCellArray.n = 0;

			pNewData = dataMem;
		}

	private:
		Array3D<QList<DataType>> grid;
		DataType *dataMem;
		int dataMemSize;
		Box<CoordinateType> volume;
		CoordinateType cellSize;
		CoordinateType cellSize2;
		DataType *pNewData;
		int nCells;
		int cellMemSize;
		CoordinateType halfCellSize;
		DataType **dataBuff;
		int iOutCell;
		Array<int> neighborCellArray;
		Array<int> activeCellArray;
		bool *bActiveCell;
	};
}