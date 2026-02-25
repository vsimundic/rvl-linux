struct RVLRECT		// If the order of the fields changes,
					// then function RVLCrop2DLine() must be corrected!
{
  int left;
  int right;
  int top;
  int bottom;
};

inline BOOL IsInside(RVLRECT &Rect, int x, int y)
{
	if(x < Rect.left)
		return FALSE;

	if(x > Rect.right)
		return FALSE;

	if(y < Rect.top)
		return FALSE;

	if(y > Rect.bottom)
		return FALSE;

	return TRUE;
};

inline BOOL Overlap(RVLRECT *pRect1, RVLRECT *pRect2)
{
	int dTopTop = pRect1->top - pRect2->top;
	int dBottomTop = pRect1->bottom - pRect2->top;
	int dTopBottom = pRect1->top - pRect2->bottom;
	int dBottomBottom = pRect1->bottom - pRect2->bottom;
	int dLeftLeft = pRect1->left - pRect2->left;
	int dRightLeft = pRect1->right - pRect2->left;
	int dLeftRight = pRect1->left - pRect2->right;
	int dRightRight = pRect1->right - pRect2->right;
	BOOL bTopOverlap = dTopTop * dBottomTop <= 0;
	BOOL bBottomOverlap = dTopBottom * dBottomBottom <= 0;
	BOOL bLeftOverlap = dLeftLeft * dRightLeft <= 0;
	BOOL bRightOverlap = dLeftRight * dRightRight <= 0;

	if(!bTopOverlap && !bBottomOverlap)
		if(dTopTop * dTopBottom > 0)
			return FALSE;

	if(!bLeftOverlap && !bRightOverlap)
		if(dLeftLeft * dLeftRight > 0)
			return FALSE;

	return TRUE;
};

inline void Add2Rect(RVLRECT &Rect, int x, int y)
{
	if(x < Rect.left)
		Rect.left = x;

	if(x > Rect.right)
		Rect.right = x;

	if(y < Rect.top)
		Rect.top = y;

	if(y > Rect.bottom)
		Rect.bottom = y;
};

inline void GetXZRect(RVLRECT &Rect, double *Polygon, int n)
{
	int i;

	Rect.left = Rect.right = (int)(Polygon[0] + 0.5);
	Rect.top = Rect.bottom = (int)(Polygon[2] + 0.5);

	double *pPoint = Polygon + 3;
	int x, z;

	for(i = 1; i < n; i ++)
	{
		x = (int)(*pPoint + 0.5);
		pPoint += 2;
		z = (int)(*pPoint + 0.5);
		pPoint++;

		Add2Rect(Rect, x, z);
	}
};

inline void GetRect(RVLRECT &Rect, int *Polygon, int n)
{
  	int i;

	Rect.left = Rect.right = Polygon[0];
	Rect.top = Rect.bottom = Polygon[1];

	int *pPoint = Polygon + 2;
	int u, v;

	for(i = 1; i < n; i ++)
	{
		u = *(pPoint++);
		v = *(pPoint++);

		Add2Rect(Rect, u, v);
	}
};


