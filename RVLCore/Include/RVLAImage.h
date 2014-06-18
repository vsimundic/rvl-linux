

class CRVLAImage
{
public:
	CRVLMem *m_pMem, *m_pMem2;
	int m_Width, m_Height;
	int m_ElementSize;
	RVLAPIX *m_pPix;
	BOOL m_bOwnData;
	int m_dpNeighbor4[4];
	int m_dpNeighbor8[8];
	int m_NeighborLimit[4];
	int m_dpContourFollow[4][4];
	BYTE m_idIDirection[3][3];
	CRVLCamera *m_pCameraL, *m_pCameraR;
	CRVLC2D m_C2DRegion;
	CRVLC2D m_C2DRegion2;
	CRVLC2D m_C2DRegion3;
	CRVLEDT m_EDT;
	RVLEDT_PIX_ARRAY m_EDTImage;
	RVLEDT_PIX_ARRAY m_EDTDisparityImage;
	RVLRECT *m_pROI;
	CRVLMPtrChain m_RelList;
	char *m_iNeighbor;
	CRVL2DRegion2 **m_2DRegionMap;
	RVLAPIX_2DREGION_PTR *m_2DRegionMap2;
	int *m_DistanceLookUpTable;

private:
	char *m_iNeighborMem;

public:
	void Init();
	void Clear();
	void Create();
	void Create(unsigned char *PixArray);
	void CreateBorder();
	CRVLAImage();
	virtual ~CRVLAImage();
};



