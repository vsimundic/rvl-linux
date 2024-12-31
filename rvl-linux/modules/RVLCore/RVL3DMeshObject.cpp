#include "RVLCore.h"
#include "RVLEDT.h"
#include "RVLC2D.h"
#include "RVL2DLine3.h"
#include "RVL3DLine2.h"
#include "RVL3DContour.h"
#include "RVL2DRegion2.h"
#include "RVL3DSurface2.h"
#include "RVLDelaunay.h"
#include "RVL3DMeshObject.h"
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
using namespace std;

#define RVLPSD_SAVE_MESH_FLAG_HULL					0x00000001


CRVL3DMeshObject::CRVL3DMeshObject()
{
	this->Init();
}

CRVL3DMeshObject::~CRVL3DMeshObject()
{
}

void CRVL3DMeshObject::Init()
{
	this->m_FaceList = NULL;
	this->m_noFaces = 0;
	this->m_noVertices = 0;
	this->m_noUsedColorPts = 0;
	this->m_noUsedLbpPts = 0;
	this->m_histRGB = NULL;
	this->m_histRGB_base[0] = 0.0;
	this->m_histRGB_base[1] = 0.0;
	this->m_histRGB_base[2] = 0.0;
	this->m_noHistDimensions = 0;
	this->m_ColorSystem = 0;
	this->m_rgbHOG9 = NULL;
	this->m_TextureLabel = 0;
	this->m_lbpR = 0;
	this->m_lbpP = 0;
	this->m_LBP = NULL;
	this->m_LBP_RIU = NULL;
	this->m_LBP_RIU_VAR = NULL;
	//this->m_shapeD2 = NULL;
	this->m_ADSD = NULL;
	this->m_ShapeLabel = 0;
	this->rootMeshObject = NULL;
	this->parentMeshObject = NULL;
	this->m_ChildMeshObjects = NULL;
	this->m_Ellipsoid = NULL;
	//this->m_TextonList = NULL;
	this->m_TextonHist = NULL;
}

void CRVL3DMeshObject::InitParent()
{
	//this->m_ChildMeshObjects = new RVLQLIST[1];
	RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST, this->m_ChildMeshObjects)
	RVLQLIST_INIT(this->m_ChildMeshObjects);
	//this->m_Pose = new CRVL3DPose;
}

void CRVL3DMeshObject::InitChild()
{
	//this->m_FaceList = new RVLQLIST[1];
	RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST, this->m_FaceList)
	RVLQLIST_INIT(this->m_FaceList);
	//this->m_histRGB = new RVLQLIST[1];
	RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST, this->m_histRGB)
	RVLQLIST_INIT(this->m_histRGB);
	//this->m_rgbHOG9 = new RVLQLIST[1];
	RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST, this->m_rgbHOG9)
	RVLQLIST_INIT(this->m_rgbHOG9);
	//this->m_shapeD2 = new RVLQLIST[1];
	//RVLQLIST_INIT(this->m_shapeD2);
	//this->m_ADSD = new float[9];
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem, float, 9, this->m_ADSD)
	memset(this->m_ADSD, 0, 9 * sizeof(float));
	//this->m_Ellipsoid = new double[3];
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem, double, 3, this->m_Ellipsoid)
	memset(this->m_Ellipsoid, 0, 3 * sizeof(double));
	//LBP deskriptori
	RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST, this->m_LBP)
	RVLQLIST_INIT(this->m_LBP);
	RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST, this->m_LBP_RIU)
	RVLQLIST_INIT(this->m_LBP_RIU);
	RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST, this->m_LBP_RIU_VAR)
	RVLQLIST_INIT(this->m_LBP_RIU_VAR);
	//RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST, this->m_TextonList)
	//RVLQLIST_INIT(this->m_TextonList);
	RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST, this->m_TextonHist)
	RVLQLIST_INIT(this->m_TextonHist);

	//this->m_Pose = new CRVL3DPose;
}

//void CRVL3DMeshObject::RVLCalculateRGBHist(IplImage* rgbImg)
//{
//	CRVL2DRegion2 *pTriangle;
//	RVL3DPOINT2 **ppPt, **pPtArrayEnd;
//	RVL3DPOINT2 *pPt;
//	int iPix;
//	int label;
//	int r = 0, g = 0, b = 0, rgb_x = 0, rgb_y = 0;
//	int w = rgbImg->width;
//	int h = rgbImg->height;
//	//ushort *histRGB16 = new ushort[4096];
//	//ushort *objHistRGB16 = new ushort[4096];
//	//memset(objHistRGB16, 0, 4096 * sizeof(ushort));
//	int *histRGB16 = new int[4096];
//	int *objHistRGB16 = new int[4096];
//	memset(objHistRGB16, 0, 4096 * sizeof(int));
//
//	uchar *ptr;
//	RVLQLIST_PTR_ENTRY *pElement;
//	pElement = (RVLQLIST_PTR_ENTRY*)this->m_FaceList->pFirst;
//	
//	int *sortIdx = new int[4096];	//Potrebno za sortiranje
//
//	while(pElement)
//	{
//		pTriangle = (CRVL2DRegion2 *)(pElement->Ptr);
//
//		if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
//			continue;
//
//		label = pTriangle->m_Label;
//
//		pPtArrayEnd = pTriangle->m_pPoint3DArray + pTriangle->m_n3DPts;
//
//		//memset(histRGB16, 0, 4096 * sizeof(ushort));
//		memset(histRGB16, 0, 4096 * sizeof(int));
//
//		pTriangle->m_histRGB16 = new RVLQLIST[1];
//		RVLQLIST_INIT(pTriangle->m_histRGB16);
//
//		for(ppPt = pTriangle->m_pPoint3DArray; ppPt < pPtArrayEnd; ppPt++)
//		{
//			pPt = *ppPt;
//
//			iPix = pPt->iPix;
//			//ako je maska postavljena i nije dozvoljen idemo dalje
//			if (pTriangle->m_uPixMask && pTriangle->m_uPixMask[iPix])
//				continue;
//			//Povecavamo broj koristenih piksela s bojom
//			this->m_noUsedColorPts++;
//			//Provjeriti
//			rgb_y = pPt->iPixRGB / w;
//			rgb_x = pPt->iPixRGB - rgb_y * w;
//			if (rgb_y != 0)
//				rgb_x --;
//			ptr = (uchar*)(rgbImg->imageData + rgb_y * rgbImg->widthStep);
//			if (rgbImg->channelSeq[0] == 'B')
//			{
//				b = ptr[rgb_x * 3];
//				g = ptr[rgb_x * 3 + 1];
//				r = ptr[rgb_x * 3 + 2];
//			}
//			else
//			{
//				r = ptr[rgb_x * 3];
//				g = ptr[rgb_x * 3 + 1];
//				b = ptr[rgb_x * 3 + 2];
//			}
//
//			histRGB16[(int)(floor(r / 16.0) * 16 * 16 + floor(g / 16.0) * 16 + floor(b / 16.0))]++;	// i = r/16 * 16^2 + g/16 * 16 + b/16
//			
//		}
//		//SORTING
//		Sort(histRGB16, sortIdx, 4096);
//		for(int i = 4095; i >= 0; i--) //zato jer je sortirao od manjeg prema vecem
//		{
//			/*PRIJE SORTIRANJA! if (histRGB16[i] != 0)
//			{
//				RVLQLIST_HIST_ENTRY_SHORT *pHistEntry = new RVLQLIST_HIST_ENTRY_SHORT[1];
//				pHistEntry->adr = i;
//				pHistEntry->value = (ushort)histRGB16[i];
//				RVLQLIST_ADD_ENTRY(pTriangle->m_histRGB16, pHistEntry);
//				//Postavljamo od objekta histogram
//				objHistRGB16[i] += histRGB16[i];
//			}*/
//			if (histRGB16[i] == 0)	//Kod prvog nailaska na nulu prekidamo jer su sve ostalo nule
//				break;
//			RVLQLIST_HIST_ENTRY_SHORT *pHistEntry = new RVLQLIST_HIST_ENTRY_SHORT[1];
//			pHistEntry->adr = sortIdx[i];
//			pHistEntry->value = (ushort)histRGB16[i];
//			RVLQLIST_ADD_ENTRY(pTriangle->m_histRGB16, pHistEntry);
//			//Postavljamo od objekta histogram
//			objHistRGB16[sortIdx[i]] += histRGB16[i];
//		}
//
//		pElement = (RVLQLIST_PTR_ENTRY*)pElement->pNext;
//	}
//
//	//ako postoji stari histogram, brisemo je (POTREBNO SAMO DOK SE NEPOVEZE SUSTAV S MEM strukturama)
//	/*SUMNJIVO if (this->m_histRGB16->pFirst)
//	{
//		RVLQLIST_HIST_ENTRY_SHORT *pEntry = (RVLQLIST_HIST_ENTRY_SHORT*)this->m_histRGB16->pFirst;
//		while(pEntry)
//		{
//			RVLQLIST_HIST_ENTRY_SHORT *pEntryD;
//			pEntryD = pEntry;
//			pEntry = new RVLQLIST_HIST_ENTRY_SHORT[1];
//			pEntry = (RVLQLIST_HIST_ENTRY_SHORT*)pEntryD->pNext;
//			delete pEntryD;
//		}
//		delete pEntry;
//		RVLQLIST_INIT(this->m_histRGB16);
//	}*/
//	Sort(objHistRGB16, sortIdx, 4096);
//	//Postavljamo histogram i azuriramo roditeljski ako postoji(AZURIRANJE JE NETESTIRANO!!!)
//	if (((CRVL3DMeshObject*)this->parentMeshObject)->m_histRGB16)
//	{
//		RVLQLIST_HIST_ENTRY_SHORT *pEntryT;
//		RVLQLIST_HIST_ENTRY_SHORT *pEntryP;
//		RVLQLIST *parentRGBHist;
//		parentRGBHist = (RVLQLIST*)((CRVL3DMeshObject*)this->parentMeshObject)->m_histRGB16;
//		pEntryT = (RVLQLIST_HIST_ENTRY_SHORT*)(parentRGBHist)->pFirst;
//		pEntryP = pEntryT;
//		int *parentObjHistRGB16 = new int[4096];
//		memset(parentObjHistRGB16, 0, 4096 * sizeof(int));
//		while(pEntryT)
//		{
//			pEntryP = pEntryT;
//			parentObjHistRGB16[pEntryP->adr] = pEntryP->value; 
//			pEntryT = (RVLQLIST_HIST_ENTRY_SHORT*)pEntryP->pNext;
//			delete pEntryP;	//SUMNJIVO!!!!!!!!!
//		}
//		for (int i = 0; i < 4096; i++)
//			parentObjHistRGB16[i] += objHistRGB16[sortIdx[i]];
//		int *sortIdxP = new int[4096];	//Potrebno za sortiranje
//		Sort(parentObjHistRGB16, sortIdxP, 4096);
//
//		for(int i = 4095; i >= 0; i--) //zato jer je sortirao od manjeg prema vecem
//		{
//			/*PRIJE SORTIRANJA! if (objHistRGB16[i] != 0)
//			{
//				RVLQLIST_HIST_ENTRY_SHORT *pHistEntry = new RVLQLIST_HIST_ENTRY_SHORT[1];
//				pHistEntry->adr = i;
//				pHistEntry->value = (ushort)objHistRGB16[i];
//				RVLQLIST_ADD_ENTRY(this->m_histRGB16, pHistEntry);
//
//				//Provjeravamo s parentom
//				while (pEntryT->adr < i)
//				{
//					pEntryP = pEntryT;
//					pEntryT = (RVLQLIST_HIST_ENTRY_SHORT*)pEntryT->pNext;
//				}
//				if (pEntryT->adr = i)
//					pEntryT->value += (ushort)objHistRGB16[i];
//				else if (pEntryT->adr > i)
//				{
//					RVLQLIST_HIST_ENTRY_SHORT *pHistNewEntry = new RVLQLIST_HIST_ENTRY_SHORT[1];
//					pHistNewEntry->adr = i;
//					pHistNewEntry->value = (ushort)objHistRGB16[i];
//					RVLQLIST_INSERT_ENTRY(parentRGBHist, pEntryP, pEntryT, pHistNewEntry);
//				}
//			}*/
//			if (objHistRGB16[i] == 0)	//Kod prvog nailaska na nulu prekidamo jer su sve ostalo nule
//				break;
//			RVLQLIST_HIST_ENTRY_SHORT *pHistEntry = new RVLQLIST_HIST_ENTRY_SHORT[1];
//			pHistEntry->adr = sortIdx[i];
//			pHistEntry->value = (ushort)objHistRGB16[i];
//			RVLQLIST_ADD_ENTRY(this->m_histRGB16, pHistEntry);	
//		}
//		//postavljamo parenta
//		for(int i = 4095; i >= 0; i--)	//zato jer je sortirao od manjeg prema vecem
//		{
//			if (parentObjHistRGB16[i] == 0)	//Kod prvog nailaska na nulu prekidamo jer su sve ostalo nule
//				break;
//			RVLQLIST_HIST_ENTRY_SHORT *pHistEntry = new RVLQLIST_HIST_ENTRY_SHORT[1];
//			pHistEntry->adr = sortIdxP[i];
//			pHistEntry->value = (ushort)parentObjHistRGB16[i];
//			RVLQLIST_ADD_ENTRY(parentRGBHist, pHistEntry);		
//		}
//		delete [] sortIdxP;
//		delete [] parentObjHistRGB16;
//	}
//	else
//	{
//		for(int i = 4095; i >= 0; i--) // zato jer je sortirao od manjeg prema vecem
//		{
//			/*PRIJE SORTIRANJA if (objHistRGB16[i] != 0)
//			{
//				RVLQLIST_HIST_ENTRY_SHORT *pHistEntry = new RVLQLIST_HIST_ENTRY_SHORT[1];
//				pHistEntry->adr = i;
//				pHistEntry->value = (ushort)objHistRGB16[i];
//				RVLQLIST_ADD_ENTRY(this->m_histRGB16, pHistEntry);
//			}*/
//			if (objHistRGB16[i] == 0)
//				break;
//			RVLQLIST_HIST_ENTRY_SHORT *pHistEntry = new RVLQLIST_HIST_ENTRY_SHORT[1];
//			pHistEntry->adr = sortIdx[i];
//			pHistEntry->value = (ushort)objHistRGB16[i];
//			RVLQLIST_ADD_ENTRY(this->m_histRGB16, pHistEntry);
//
//		}
//	}
//	//oslobaðamo memoriju
//	delete [] sortIdx;
//	delete [] histRGB16; 
//	delete [] objHistRGB16;
//}
void CRVL3DMeshObject::RVLCalculateRGBHist(IplImage* rgbImg, float* histBase, bool soft)
{
	int noBins = (int)histBase[0] * histBase[1] * histBase[2];
	float ratioR = 256 / histBase[0]; //how many levels are per bin
	float ratioG = 256 / histBase[1]; //how many levels are per bin
	float ratioB = 256 / histBase[2]; //how many levels are per bin
	CRVL2DRegion2 *pTriangle;
	RVL3DPOINT2 **ppPt, **pPtArrayEnd;
	RVL3DPOINT2 *pPt;
	int iPix;
	int label;
	int r = 0, g = 0, b = 0, rgb_x = 0, rgb_y = 0;
	int w = rgbImg->width;
	int h = rgbImg->height;
	int adr;

	int updateBins[8][3];	//Needed for soft histograms, coordinates for updated bins
	memset(updateBins, 0, 8 * 3 * sizeof(int));
	int bins2Update = 0;	//Needed for soft histograms, number of bins needed to update
	int binCentar[3];	//Needed for soft histograms, closest bin centar
	int bin[3];	//Needed for soft histograms, central bin coordinates
	int valuePos[3];	//current value above, below or on bin centar, per dimension(R,G,B)
	float binWeightCoeff[8][3];	//Needed for soft histograms, calculated coefficients for faster analysis
	float weightTypes[2][3];	//Needed for soft histograms, there can be only two weights, offset by 0 or +/- 1
	memset(binWeightCoeff, 0, 8 * 3 * sizeof(float));

	void** histRGB;// = new void*[noBins];
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, void*, noBins, histRGB);
	RVLQLIST *histRGBList;// = new RVLQLIST[1];
	RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST, histRGBList);

	void** objHistRGB;// = new void*[noBins];
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, void*, noBins, objHistRGB);
	for(int i = 0; i < noBins; i++)
	{
		if (objHistRGB[i])
			objHistRGB[i] = NULL;
	}
	RVLQLIST *objHistRGBList;// = new RVLQLIST[1];
	RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST, objHistRGBList);
	RVLQLIST_INIT(objHistRGBList);

	uchar *ptr;
	RVLQLIST_PTR_ENTRY *pElement;
	pElement = (RVLQLIST_PTR_ENTRY*)this->m_FaceList->pFirst;

	while(pElement)
	{
		pTriangle = (CRVL2DRegion2 *)(pElement->Ptr);

		if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		label = pTriangle->m_Label;

		pPtArrayEnd = pTriangle->m_pPoint3DArray + pTriangle->m_n3DPts;

		for(int i = 0; i < noBins; i++)
		{
			if (histRGB[i])
				histRGB[i] = NULL;
		}
		RVLQLIST_INIT(histRGBList);	//resetiramo listu 

		//pTriangle->m_histRGB;// = new RVLQLIST[1];
		RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST, pTriangle->m_histRGB);
		RVLQLIST_INIT(pTriangle->m_histRGB);
		pTriangle->m_histRGB_base[0] = histBase[0];
		pTriangle->m_histRGB_base[1] = histBase[1];
		pTriangle->m_histRGB_base[2] = histBase[2];
		if ((rgbImg->channelSeq[0] == 'B') || (rgbImg->channelSeq[0] == 'R'))
			pTriangle->m_ColorSystem = RVL_MESH_COLOR_RGB;
		else if (rgbImg->channelSeq[0] == 'X')
			pTriangle->m_ColorSystem = RVL_MESH_COLOR_XYZ;

		for(ppPt = pTriangle->m_pPoint3DArray; ppPt < pPtArrayEnd; ppPt++)
		{
			pPt = *ppPt;

			iPix = pPt->iPix;
			//ako je maska postavljena i nije dozvoljen idemo dalje
			if (pTriangle->m_uPixMask && (pTriangle->m_uPixMask[iPix] == 1))
				continue;
			//Povecavamo broj koristenih piksela s bojom
			this->m_noUsedColorPts++;
			//Provjeriti
			rgb_y = pPt->iPixRGB / w;
			rgb_x = pPt->iPixRGB - rgb_y * w;
			if (rgb_y != 0)
				rgb_x --;
			ptr = (uchar*)(rgbImg->imageData + rgb_y * rgbImg->widthStep);
			if (rgbImg->channelSeq[0] == 'B')
			{
				b = ptr[rgb_x * 3];
				g = ptr[rgb_x * 3 + 1];
				r = ptr[rgb_x * 3 + 2];
			}
			else
			{
				r = ptr[rgb_x * 3];
				g = ptr[rgb_x * 3 + 1];
				b = ptr[rgb_x * 3 + 2];
			}

			if(!soft)
			{
				adr = (int)(floor(r / ratioR) * histBase[1] * histBase[2] + floor(g / ratioG) * histBase[2] + floor(b / ratioB)); // i = r/(256/16) * 16^2 + g/(256/16) * 16 + b/(256/16)
				//Normalized RGB adr = (int)(floor(((float)r / (float)(r + g + b)) / ratio) * histBase * histBase + floor(((float)g / (float)(r + g + b)) / ratio) * histBase + floor(((float)b / (float)(r + g + b)) / ratio));
				if (histRGB[adr])
				{
					//azuriramo lokalni trikutic
					((RVLQLIST_HIST_ENTRY*)histRGB[adr])->value++;
					//azuriramo objekt
					((RVLQLIST_HIST_ENTRY*)objHistRGB[adr])->value++;
				}
				else
				{
					//stvaramo novi element koji ce biti i u arrayu i u povezanom popisu
					RVLQLIST_HIST_ENTRY *pHistEntryT;// = new RVLQLIST_HIST_ENTRY_SHORT[1];
					RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST_HIST_ENTRY, pHistEntryT);
					pHistEntryT->adr = adr;
					pHistEntryT->value = 1;
					histRGB[adr] = pHistEntryT;
					RVLQLIST_ADD_ENTRY(histRGBList, pHistEntryT);
					
					if (!objHistRGB[adr])
					{
						RVLQLIST_HIST_ENTRY *pHistEntryO;// = new RVLQLIST_HIST_ENTRY_SHORT[1];
						RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST_HIST_ENTRY, pHistEntryO);
						pHistEntryO->adr = adr;
						pHistEntryO->value = 1;
						objHistRGB[adr] = pHistEntryO;
						RVLQLIST_ADD_ENTRY(objHistRGBList, pHistEntryO);
					}
					else
						((RVLQLIST_HIST_ENTRY*)objHistRGB[adr])->value++;	//azuriramo objekt
				}
			}
			else
			{
				//closest bin centar
				bin[0] = (int)floor(r / ratioR);
				bin[1] = (int)floor(g / ratioG);
				bin[2] = (int)floor(b / ratioB);
				binCentar[0] = (int)(bin[0] * ratioR + ratioR / 2);
				binCentar[1] = (int)(bin[1] * ratioG + ratioG / 2);
				binCentar[2] = (int)(bin[2] * ratioB + ratioB / 2);
				
				//is current value above bin centar per dimension
				if (r > binCentar[0])
					valuePos[0] = 1;
				else if (r == binCentar[0])
					valuePos[0] = 0;
				else if (r < binCentar[0])
					valuePos[0] = -1;

				if (g > binCentar[1])
					valuePos[1] = 1;
				else if (g == binCentar[1])
					valuePos[1] = 0;
				else if (g < binCentar[1])
					valuePos[1] = -1;

				if (b > binCentar[2])
					valuePos[2] = 1;
				else if (b == binCentar[2])
					valuePos[2] = 0;
				else if (b < binCentar[2])
					valuePos[2] = -1;

				//Finding which bin to update
				updateBins[0][0] = bin[0];
				updateBins[0][1] = bin[1];
				updateBins[0][2] = bin[2];
				//Calculating weight types
				//'0' offset
				weightTypes[0][0] = (float)(1.0 - abs((float)(r - (updateBins[0][0] * ratioR + ratioR / 2.0))) / ratioR);
				weightTypes[0][1] = (float)(1.0 - abs((float)(g - (updateBins[0][1] * ratioG + ratioG / 2.0))) / ratioG);
				weightTypes[0][2] = (float)(1.0 - abs((float)(b - (updateBins[0][2] * ratioB + ratioB / 2.0))) / ratioB);
				//'+/- 1' offset
				weightTypes[1][0] = (float)(1.0 - abs((float)(r - ((bin[0] + valuePos[0]) * ratioR + ratioR / 2.0))) / ratioR);
				weightTypes[1][1] = (float)(1.0 - abs((float)(g - ((bin[1] + valuePos[1]) * ratioG + ratioG / 2.0))) / ratioG);
				weightTypes[1][2] = (float)(1.0 - abs((float)(b - ((bin[2] + valuePos[2]) * ratioB + ratioB / 2.0))) / ratioB);
				
				/*binWeightCoeff[0][0] = (float)(1.0 - abs((float)(r - (updateBins[0][0] * ratio + ratio / 2.0))) / ratio);
				binWeightCoeff[0][1] = (float)(1.0 - abs((float)(g - (updateBins[0][1] * ratio + ratio / 2.0))) / ratio);
				binWeightCoeff[0][2] = (float)(1.0 - abs((float)(b - (updateBins[0][2] * ratio + ratio / 2.0))) / ratio);*/
				binWeightCoeff[0][0] = weightTypes[0][0];
				binWeightCoeff[0][1] = weightTypes[0][1];
				binWeightCoeff[0][2] = weightTypes[0][2];
				bins2Update = 1;
				if (!((valuePos[0] == 0) && (valuePos[1] == 0) && (valuePos[2] == 0)))	//current color is in bin centar
				{
					//first coordinate
					if (valuePos[0] != 0)
					{
						updateBins[bins2Update][0] = bin[0] + valuePos[0];
						if ((updateBins[bins2Update][0] >= 0) && (updateBins[bins2Update][0] < histBase[0]))
						{
							updateBins[bins2Update][1] = bin[1];
							updateBins[bins2Update][2] = bin[2];
							/*binWeightCoeff[bins2Update][0] = (float)(1.0 - abs((float)(r - (updateBins[bins2Update][0] * ratio + ratio / 2.0))) / ratio);
							binWeightCoeff[bins2Update][1] = (float)(1.0 - abs((float)(g - (updateBins[bins2Update][1] * ratio + ratio / 2.0))) / ratio);
							binWeightCoeff[bins2Update][2] = (float)(1.0 - abs((float)(b - (updateBins[bins2Update][2] * ratio + ratio / 2.0))) / ratio);*/
							binWeightCoeff[bins2Update][0] = weightTypes[abs(bin[0] - updateBins[bins2Update][0])][0];
							binWeightCoeff[bins2Update][1] = weightTypes[abs(bin[1] - updateBins[bins2Update][1])][1];
							binWeightCoeff[bins2Update][2] = weightTypes[abs(bin[2] - updateBins[bins2Update][2])][2];
							bins2Update++;

							//first & second coordinate
							updateBins[bins2Update][1] = bin[1] + valuePos[1];
							if ((updateBins[bins2Update][1] >= 0) && (updateBins[bins2Update][1] < histBase[1]))
							{
								updateBins[bins2Update][0] = bin[0] + valuePos[0];
								updateBins[bins2Update][2] = bin[2];
								/*binWeightCoeff[bins2Update][0] = (float)(1.0 - abs((float)(r - (updateBins[bins2Update][0] * ratio + ratio / 2.0))) / ratio);
								binWeightCoeff[bins2Update][1] = (float)(1.0 - abs((float)(g - (updateBins[bins2Update][1] * ratio + ratio / 2.0))) / ratio);
								binWeightCoeff[bins2Update][2] = (float)(1.0 - abs((float)(b - (updateBins[bins2Update][2] * ratio + ratio / 2.0))) / ratio);*/
								binWeightCoeff[bins2Update][0] = weightTypes[abs(bin[0] - updateBins[bins2Update][0])][0];
								binWeightCoeff[bins2Update][1] = weightTypes[abs(bin[1] - updateBins[bins2Update][1])][1];
								binWeightCoeff[bins2Update][2] = weightTypes[abs(bin[2] - updateBins[bins2Update][2])][2];
								bins2Update++;
							}

							//first & third coordinate
							updateBins[bins2Update][2] = bin[2] + valuePos[2];
							if ((updateBins[bins2Update][2] >= 0)  && (updateBins[bins2Update][2] < histBase[2]))
							{
								updateBins[bins2Update][0] = bin[0] + valuePos[0];
								updateBins[bins2Update][1] = bin[1];
								/*binWeightCoeff[bins2Update][0] = (float)(1.0 - abs((float)(r - (updateBins[bins2Update][0] * ratio + ratio / 2.0))) / ratio);
								binWeightCoeff[bins2Update][1] = (float)(1.0 - abs((float)(g - (updateBins[bins2Update][1] * ratio + ratio / 2.0))) / ratio);
								binWeightCoeff[bins2Update][2] = (float)(1.0 - abs((float)(b - (updateBins[bins2Update][2] * ratio + ratio / 2.0))) / ratio);*/
								binWeightCoeff[bins2Update][0] = weightTypes[abs(bin[0] - updateBins[bins2Update][0])][0];
								binWeightCoeff[bins2Update][1] = weightTypes[abs(bin[1] - updateBins[bins2Update][1])][1];
								binWeightCoeff[bins2Update][2] = weightTypes[abs(bin[2] - updateBins[bins2Update][2])][2];
								bins2Update++;
							}
						}
					}

					//second coordinate
					if (valuePos[1] != 0)
					{
						updateBins[bins2Update][1] = bin[1] + valuePos[1];
						if ((updateBins[bins2Update][1] >= 0)  && (updateBins[bins2Update][1] < histBase[1]))
						{
							updateBins[bins2Update][0] = bin[0];
							updateBins[bins2Update][2] = bin[2];
							/*binWeightCoeff[bins2Update][0] = (float)(1.0 - abs((float)(r - (updateBins[bins2Update][0] * ratio + ratio / 2.0))) / ratio);
							binWeightCoeff[bins2Update][1] = (float)(1.0 - abs((float)(g - (updateBins[bins2Update][1] * ratio + ratio / 2.0))) / ratio);
							binWeightCoeff[bins2Update][2] = (float)(1.0 - abs((float)(b - (updateBins[bins2Update][2] * ratio + ratio / 2.0))) / ratio);*/
							binWeightCoeff[bins2Update][0] = weightTypes[abs(bin[0] - updateBins[bins2Update][0])][0];
							binWeightCoeff[bins2Update][1] = weightTypes[abs(bin[1] - updateBins[bins2Update][1])][1];
							binWeightCoeff[bins2Update][2] = weightTypes[abs(bin[2] - updateBins[bins2Update][2])][2];
							bins2Update++;

							//second & third coordinate
							updateBins[bins2Update][2] = bin[2] + valuePos[2];
							if ((updateBins[bins2Update][2] >= 0) && (updateBins[bins2Update][2] < histBase[2]))
							{
								updateBins[bins2Update][0] = bin[0];
								updateBins[bins2Update][1] = bin[1] + valuePos[1];
								/*binWeightCoeff[bins2Update][0] = (float)(1.0 - abs((float)(r - (updateBins[bins2Update][0] * ratio + ratio / 2.0))) / ratio);
								binWeightCoeff[bins2Update][1] = (float)(1.0 - abs((float)(g - (updateBins[bins2Update][1] * ratio + ratio / 2.0))) / ratio);
								binWeightCoeff[bins2Update][2] = (float)(1.0 - abs((float)(b - (updateBins[bins2Update][2] * ratio + ratio / 2.0))) / ratio);*/
								binWeightCoeff[bins2Update][0] = weightTypes[abs(bin[0] - updateBins[bins2Update][0])][0];
								binWeightCoeff[bins2Update][1] = weightTypes[abs(bin[1] - updateBins[bins2Update][1])][1];
								binWeightCoeff[bins2Update][2] = weightTypes[abs(bin[2] - updateBins[bins2Update][2])][2];
								bins2Update++;
							}
						}
					}
					//third coordinate
					if (valuePos[2] != 0)
					{
						updateBins[bins2Update][2] = bin[2] + valuePos[2];
						if ((updateBins[bins2Update][2] >= 0) && (updateBins[bins2Update][2] < histBase[2]))
						{
							updateBins[bins2Update][0] = bin[0];
							updateBins[bins2Update][1] = bin[1];
							/*binWeightCoeff[bins2Update][0] = (float)(1.0 - abs((float)(r - (updateBins[bins2Update][0] * ratio + ratio / 2.0))) / ratio);
							binWeightCoeff[bins2Update][1] = (float)(1.0 - abs((float)(g - (updateBins[bins2Update][1] * ratio + ratio / 2.0))) / ratio);
							binWeightCoeff[bins2Update][2] = (float)(1.0 - abs((float)(b - (updateBins[bins2Update][2] * ratio + ratio / 2.0))) / ratio);*/
							binWeightCoeff[bins2Update][0] = weightTypes[abs(bin[0] - updateBins[bins2Update][0])][0];
							binWeightCoeff[bins2Update][1] = weightTypes[abs(bin[1] - updateBins[bins2Update][1])][1];
							binWeightCoeff[bins2Update][2] = weightTypes[abs(bin[2] - updateBins[bins2Update][2])][2];
							bins2Update++;
						}
					}

					//All three coordinates
					updateBins[bins2Update][0] = bin[0] + valuePos[0];
					updateBins[bins2Update][1] = bin[1] + valuePos[1];
					updateBins[bins2Update][2] = bin[2] + valuePos[2];
					if ((updateBins[bins2Update][0] >= 0) && (updateBins[bins2Update][0] < histBase[0]) && (updateBins[bins2Update][1] >= 0) && (updateBins[bins2Update][1] < histBase[1]) && (updateBins[bins2Update][2] >= 0) && (updateBins[bins2Update][2] < histBase[2]))
					{
						/*binWeightCoeff[bins2Update][0] = (float)(1.0 - abs((float)(r - (updateBins[bins2Update][0] * ratio + ratio / 2.0))) / ratio);
						binWeightCoeff[bins2Update][1] = (float)(1.0 - abs((float)(g - (updateBins[bins2Update][1] * ratio + ratio / 2.0))) / ratio);
						binWeightCoeff[bins2Update][2] = (float)(1.0 - abs((float)(b - (updateBins[bins2Update][2] * ratio + ratio / 2.0))) / ratio);*/
						binWeightCoeff[bins2Update][0] = weightTypes[abs(bin[0] - updateBins[bins2Update][0])][0];
						binWeightCoeff[bins2Update][1] = weightTypes[abs(bin[1] - updateBins[bins2Update][1])][1];
						binWeightCoeff[bins2Update][2] = weightTypes[abs(bin[2] - updateBins[bins2Update][2])][2];
						bins2Update++;
					}
				}

				//Updating all taged bins
				for (int i = 0; i < bins2Update; i++)
				{
					adr = (int)(updateBins[i][0] * histBase[1] * histBase[2] + updateBins[i][1] * histBase[2] + updateBins[i][2]); // i = r/(256/16) * 16^2 + g/(256/16) * 16 + b/(256/16)

					if (histRGB[adr])
					{
						//azuriramo lokalni trikutic
						((RVLQLIST_HIST_ENTRY*)histRGB[adr])->value += binWeightCoeff[i][0] * binWeightCoeff[i][1] * binWeightCoeff[i][2];
						//azuriramo objekt
						((RVLQLIST_HIST_ENTRY*)objHistRGB[adr])->value += binWeightCoeff[i][0] * binWeightCoeff[i][1] * binWeightCoeff[i][2];
					}
					else
					{
						//stvaramo novi element koji ce biti i u arrayu i u povezanom popisu
						RVLQLIST_HIST_ENTRY *pHistEntryT;// = new RVLQLIST_HIST_ENTRY_SHORT[1];
						RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST_HIST_ENTRY, pHistEntryT);
						pHistEntryT->adr = adr;
						pHistEntryT->value = binWeightCoeff[i][0] * binWeightCoeff[i][1] * binWeightCoeff[i][2];
						histRGB[adr] = pHistEntryT;
						RVLQLIST_ADD_ENTRY(histRGBList, pHistEntryT);
						
						if (!objHistRGB[adr])
						{
							RVLQLIST_HIST_ENTRY *pHistEntryO;// = new RVLQLIST_HIST_ENTRY_SHORT[1];
							RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST_HIST_ENTRY, pHistEntryO);
							pHistEntryO->adr = adr;
							pHistEntryO->value = binWeightCoeff[i][0] * binWeightCoeff[i][1] * binWeightCoeff[i][2];
							objHistRGB[adr] = pHistEntryO;
							RVLQLIST_ADD_ENTRY(objHistRGBList, pHistEntryO);
						}
						else
							((RVLQLIST_HIST_ENTRY*)objHistRGB[adr])->value += binWeightCoeff[i][0] * binWeightCoeff[i][1] * binWeightCoeff[i][2];	//azuriramo objekt
					}

				}

			}
			
		}
		//Stvaramo i sortiramo RGB histogram za trokut
		pTriangle->m_histRGB = InsertSortQLISTWithCopy<RVLQLIST_HIST_ENTRY>(histRGBList, RVLQLIST_DESCENDING, this->m_pClass->m_pMem0, TRUE);
		pElement = (RVLQLIST_PTR_ENTRY*)pElement->pNext;
	}
	//Stvaramo i sortiramo RGB histogram za cijeli objekt
	this->m_histRGB = InsertSortQLISTWithCopy<RVLQLIST_HIST_ENTRY>(objHistRGBList, RVLQLIST_DESCENDING, this->m_pClass->m_pMem0, TRUE);
	this->m_histRGB_base[0] = histBase[0];
	this->m_histRGB_base[1] = histBase[1];
	this->m_histRGB_base[2] = histBase[2];
	if ((rgbImg->channelSeq[0] == 'B') || (rgbImg->channelSeq[0] == 'R'))
		this->m_ColorSystem = RVL_MESH_COLOR_RGB;
	else if (rgbImg->channelSeq[0] == 'X')
		this->m_ColorSystem = RVL_MESH_COLOR_XYZ;

	//Azuriramo histogram od parenta ako postoji(NETESTIRANO!!!! STO AKO IMAJU RAZLICITE BAZE?!?!?)
	if (((CRVL3DMeshObject*)this->parentMeshObject)->m_histRGB)
	{
		RVLQLIST *pParentHistList = ((CRVL3DMeshObject*)this->parentMeshObject)->m_histRGB;
		if (pParentHistList->pFirst)
		{
			RVLQLIST_HIST_ENTRY *pParentHistEntry = (RVLQLIST_HIST_ENTRY *)pParentHistList->pFirst;
			RVLQLIST_HIST_ENTRY *pObjHistEntry = (RVLQLIST_HIST_ENTRY *)this->m_histRGB->pFirst;
			bool insert = FALSE;
			while(pObjHistEntry)
			{
				while (pParentHistEntry)
				{
					if (pObjHistEntry->adr == pParentHistEntry->adr)
						pParentHistEntry->value += pObjHistEntry->value;
					pParentHistEntry = (RVLQLIST_HIST_ENTRY *)pParentHistEntry->pNext;
					if (!pParentHistEntry)
					{
						RVLQLIST_ADD_ENTRY(pParentHistList, pObjHistEntry);
						insert = TRUE;
						break;
					}
				}
				pObjHistEntry = (RVLQLIST_HIST_ENTRY *)pObjHistEntry->pNext;
			}
			if (insert)
				((CRVL3DMeshObject*)this->parentMeshObject)->m_histRGB = InsertSortQLISTWithCopy<RVLQLIST_HIST_ENTRY>(pParentHistList, RVLQLIST_DESCENDING, this->m_pClass->m_pMem0, TRUE);

		}
		else
		{
			//Kopiraj podatke od objekta
			RVLQLIST_HIST_ENTRY *pObjHistEntry = (RVLQLIST_HIST_ENTRY *)this->m_histRGB->pFirst;
			while(pObjHistEntry)
			{
				RVLQLIST_HIST_ENTRY *pParentHistNewEntry;// = new RVLQLIST_HIST_ENTRY_SHORT[1];
				RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem0, RVLQLIST_HIST_ENTRY, pParentHistNewEntry);
				pParentHistNewEntry->adr = pObjHistEntry->adr;
				pParentHistNewEntry->value = pObjHistEntry->value;
				RVLQLIST_ADD_ENTRY(pParentHistList, pParentHistNewEntry);

				pObjHistEntry = (RVLQLIST_HIST_ENTRY *)pObjHistEntry->pNext;
			}
		}
	}		
}

void CRVL3DMeshObject::RVLCalculateRGBHOG(IplImage* rgbImg,
										  float *r_x,
										  float *r_y,
										  float *g_x,
										  float *g_y,
										  float *b_x,
										  float *b_y,
										  float maxMagnitude,
										  int HOG_TYPE)
{
	//pomocne varijable
	int h = rgbImg->height;
	int w = rgbImg->width;
	float *angle_r;// = new float[h * w];
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, float, h * w, angle_r)
	memset(angle_r, 0, h * w * sizeof(float));
	float *magnitude_r;// = new float[h * w];
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, float, h * w, magnitude_r)
	memset(magnitude_r, 0, h * w * sizeof(float));
	float *angle_g;// = new float[h * w];
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, float, h * w, angle_g)
	memset(angle_g, 0, h * w * sizeof(float));
	float *magnitude_g;// = new float[h * w];
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, float, h * w, magnitude_g)
	memset(magnitude_g, 0, h * w * sizeof(float));
	float *angle_b;// = new float[h * w];
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, float, h * w, angle_b)
	memset(angle_b, 0, h * w * sizeof(float));
	float *magnitude_b;// = new float[h * w];
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, float, h * w, magnitude_b)
	memset(magnitude_b, 0, h * w * sizeof(float));
	bool *map;// = new bool[h * w];
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, bool, h * w, map)
	memset(map, 0, h * w * sizeof(bool));
	float ang3x3_r[9];
	float mag3x3_r[9];
	float ang3x3_g[9];
	float mag3x3_g[9];
	float ang3x3_b[9];
	float mag3x3_b[9];
	float pixHOG_r[9];
	float pixHOG_g[9];
	float pixHOG_b[9];
	int adr = 0, adrD = 0;
	int rgbOrient[3];
	int rgbOrientMag[3];
	float binSizeAngle = PI / 9.0;
	float binSizeMagnitude = maxMagnitude / 10.0;
	float posAng;
	float posMag;
	int hist_bins;
	if (HOG_TYPE == RVL_MESH_HOG_1D)
		hist_bins = 3 * 9;
	else if (HOG_TYPE == RVL_MESH_HOG_2D)
		hist_bins = 3 * 9 * 10;
	else if (HOG_TYPE == RVL_MESH_HOG_3D)
		hist_bins = 9 * 9 * 9;

	CRVL2DRegion2 *pTriangle;
	RVL3DPOINT2 **ppPt, **pPtArrayEnd;
	RVL3DPOINT2 *pPt;
	int iPix, iPixD;
	int label;
	ushort *rgbHOG9;// = new ushort[hist_bins];
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, ushort, hist_bins, rgbHOG9)
	ushort *objrgbHOG9;// = new ushort[hist_bins];
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, ushort, hist_bins, objrgbHOG9)
	memset(objrgbHOG9, 0, hist_bins * sizeof(ushort));

	uchar *ptr;
	RVLQLIST_PTR_ENTRY *pElement;
	pElement = (RVLQLIST_PTR_ENTRY*)this->m_FaceList->pFirst;

	while(pElement)
	{
		pTriangle = (CRVL2DRegion2 *)(pElement->Ptr);

		if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		label = pTriangle->m_Label;

		pPtArrayEnd = pTriangle->m_pPoint3DArray + pTriangle->m_n3DPts;

		memset(rgbHOG9, 0, hist_bins * sizeof(ushort));
		//pTriangle->m_rgbHOG9 = new RVLQLIST[1];
		RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST, pTriangle->m_rgbHOG9);
		RVLQLIST_INIT(pTriangle->m_rgbHOG9);

		for(ppPt = pTriangle->m_pPoint3DArray; ppPt < pPtArrayEnd; ppPt++)
		{
			pPt = *ppPt;
			
			iPixD = pPt->iPix;
			iPix = pPt->iPixRGB;
			//ako je maska postavljena i nije dozvoljen idemo dalje
			if (pTriangle->m_uPixMask && (pTriangle->m_uPixMask[iPixD] == 1))
				continue;
			memset(pixHOG_r, 0.0, 9 * sizeof(float));
			memset(pixHOG_g, 0.0, 9 * sizeof(float));
			memset(pixHOG_b, 0.0, 9 * sizeof(float));
			//racunamo okolinu
			for (int i = 0; i < 9; i++)
			{
				//za koji piksel racunamo(hardcoded? na 320x240???)
				if ((i == 0) && (iPix != 0))
				{
					adr = iPix - rgbImg->width - 1; //gornji lijevi
					adrD = iPixD - rgbImg->width - 1;
				}
				else if ((i == 1) && (iPix > rgbImg->width))
				{
					adr = iPix - rgbImg->width; //gornji srednji
					adrD = iPixD - rgbImg->width;
				}
				else if ((i == 2) && (iPix > rgbImg->width) && ((iPix + 1) % rgbImg->width != 0))
				{
					adr = iPix - rgbImg->width + 1; //gornji desni
					adrD = iPixD - rgbImg->width + 1;
				}
				else if ((i == 3) && (iPix != rgbImg->width - 1))
				{
					adr = iPix - 1; //srednji lijevi
					adrD = iPixD - 1;
				}
				else if (i == 4)
				{
					adr = iPix; //srednji(promatrani)
					adrD = iPixD;
				}
				else if ((i == 5) && ((iPix + 1) % rgbImg->width != 0))
				{
					adr = iPix + 1; //srednji desni
					adrD = iPixD + 1;
				}
				else if ((i == 6) && (iPix % rgbImg->width != 0) && ((iPix + rgbImg->width) < rgbImg->width * rgbImg->height))
				{
					adr = iPix + rgbImg->width - 1; //donji lijevi
					adrD = iPixD + rgbImg->width - 1;
				}
				else if ((i == 7) && ((iPix + rgbImg->width) < rgbImg->width * rgbImg->height))
				{
					adr = iPix + rgbImg->width; //donji srednji
					adrD = iPixD + rgbImg->width;
				}
				else if ((i == 8) && ((iPix + rgbImg->width) < rgbImg->width * rgbImg->height) && (iPix != (rgbImg->width * rgbImg->height - 1)))
				{
					adr = iPix + rgbImg->width + 1; //donji desni
					adrD = iPixD + rgbImg->width + 1;
				}
				else
				{
					ang3x3_r[i] = 0.0;
					mag3x3_r[i] = 0.0;
					ang3x3_g[i] = 0.0;
					mag3x3_g[i] = 0.0;
					ang3x3_b[i] = 0.0;
					mag3x3_b[i] = 0.0;
					continue;
				}
				//ako nije dozvoljen idemo dalje
				if (pTriangle->m_uPixMask && (pTriangle->m_uPixMask[adrD] == 1))
				{
					ang3x3_r[i] = 0.0;
					mag3x3_r[i] = 0.0;
					ang3x3_g[i] = 0.0;
					mag3x3_g[i] = 0.0;
					ang3x3_b[i] = 0.0;
					mag3x3_b[i] = 0.0;
					continue;
				}

				if (map[adr])
				{
					ang3x3_r[i] = angle_r[adr];
					mag3x3_r[i] = magnitude_r[adr];
					ang3x3_g[i] = angle_g[adr];
					mag3x3_g[i] = magnitude_g[adr];
					ang3x3_b[i] = angle_b[adr];
					mag3x3_b[i] = magnitude_b[adr];
					continue;
				}

				angle_r[adr] = atan2(r_y[adr], r_x[adr]);
				magnitude_r[adr] = sqrt(r_x[adr] * r_x[adr] + r_y[adr] * r_y[adr]);
				angle_g[adr] = atan2(g_y[adr], g_x[adr]);
				magnitude_g[adr] = sqrt(g_x[adr] * g_x[adr] + g_y[adr] * g_y[adr]);
				angle_b[adr] = atan2(b_y[adr], b_x[adr]);
				magnitude_b[adr] = sqrt(b_x[adr] * b_x[adr] + b_y[adr] * b_y[adr]);
				map[adr] = TRUE;
				ang3x3_r[i] = angle_r[adr];
				mag3x3_r[i] = magnitude_r[adr];
				ang3x3_g[i] = angle_g[adr];
				mag3x3_g[i] = magnitude_g[adr];
				ang3x3_b[i] = angle_b[adr];
				mag3x3_b[i] = magnitude_b[adr];
			}

			//Odredujemo kojem binu pripada orjentacija i stvaramo histogram
			for (int i = 0; i < 9; i++)
			{
				if (ang3x3_r[i] < 0)
					ang3x3_r[i] += PI;
				posAng = ang3x3_r[i] / binSizeAngle;				
				pixHOG_r[(int)posAng == 9 ? (int)(posAng - 1) : (int)posAng] += mag3x3_r[i];

				if (ang3x3_g[i] < 0)
					ang3x3_g[i] += PI;
				posAng = ang3x3_g[i] / binSizeAngle;
				pixHOG_g[(int)posAng == 9 ? (int)(posAng - 1) : (int)posAng] += mag3x3_g[i];

				if (ang3x3_b[i] < 0)
					ang3x3_b[i] += PI;
				posAng = ang3x3_b[i] / binSizeAngle;
				pixHOG_b[(int)posAng == 9 ? (int)(posAng - 1) : (int)posAng] += mag3x3_b[i];
			}
			Max(pixHOG_r, 9, rgbOrient[0]);
			Max(pixHOG_g, 9, rgbOrient[1]);
			Max(pixHOG_b, 9, rgbOrient[2]);
			//povecavamo odgovarajuci bin histograma
			if (HOG_TYPE == RVL_MESH_HOG_3D)
				rgbHOG9[(int)(rgbOrient[0] * 9 * 9 + rgbOrient[1] * 9 + rgbOrient[2])]++;	// i = r_orientation/16 * 16^2 + g_orientation/16 * 16 + b_orientation/16
			else if (HOG_TYPE == RVL_MESH_HOG_1D)
			{
				//R
				rgbHOG9[rgbOrient[0]]++;
				//G
				rgbHOG9[9 + rgbOrient[1]]++;
				//B
				rgbHOG9[18 + rgbOrient[2]]++;
			}
			else if (HOG_TYPE == RVL_MESH_HOG_2D)
			{
				posMag = pixHOG_r[rgbOrient[0]] / binSizeMagnitude;				
				if ((int)posMag >= 10)
					rgbOrientMag[0] = 9;
				else
					rgbOrientMag[0] = (int)posMag;

				posMag = pixHOG_g[rgbOrient[1]] / binSizeMagnitude;
				if ((int)posMag >= 10)
					rgbOrientMag[1] = 9;
				else
					rgbOrientMag[1] = (int)posMag;

				posMag = pixHOG_b[rgbOrient[2]] / binSizeMagnitude;
				if ((int)posMag >= 10)
					rgbOrientMag[2] = 9;
				else
					rgbOrientMag[2] = (int)posMag;
				//R
				rgbHOG9[0 + rgbOrientMag[0] * 9 + rgbOrient[0]]++;
				//G
				rgbHOG9[90 + rgbOrientMag[1] * 9 + rgbOrient[1]]++;
				//B
				rgbHOG9[180 + rgbOrientMag[2] * 9 + rgbOrient[2]]++;
			}
		}

		for(int i = 0; i < hist_bins; i++)
		{
			if (rgbHOG9[i] != 0)
			{
				RVLQLIST_HIST_ENTRY_SHORT *pHistEntry;// = new RVLQLIST_HIST_ENTRY_SHORT[1];
				RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST_HIST_ENTRY_SHORT, pHistEntry);
				pHistEntry->adr = i;
				pHistEntry->value = rgbHOG9[i];
				RVLQLIST_ADD_ENTRY(pTriangle->m_rgbHOG9, pHistEntry);
				objrgbHOG9[i] += rgbHOG9[i];
			}
		}

		pElement = (RVLQLIST_PTR_ENTRY*)pElement->pNext;
	}

	//ako postoji stari histogram, brisemo je (POTREBNO SAMO DOK SE NEPOVEZE SUSTAV S MEM strukturama)
	/*SUMNJIVO if (this->m_rgbHOG9->pFirst)
	{
		RVLQLIST_HIST_ENTRY_SHORT *pEntry = (RVLQLIST_HIST_ENTRY_SHORT*)this->m_rgbHOG9->pFirst;
		while(pEntry)
		{
			RVLQLIST_HIST_ENTRY_SHORT *pEntryD;
			pEntryD = pEntry;
			pEntry = new RVLQLIST_HIST_ENTRY_SHORT[1];
			pEntry = (RVLQLIST_HIST_ENTRY_SHORT*)pEntryD->pNext;
			delete pEntryD;
		}
		delete pEntry;
		RVLQLIST_INIT(this->m_rgbHOG9);
	}*/

	//Popunjavamo listu histograma i azuriramo roditeljski ako postoji
	if (((CRVL3DMeshObject*)this->parentMeshObject)->m_rgbHOG9)
	{
		RVLQLIST_HIST_ENTRY_SHORT *pEntryT;
		RVLQLIST_HIST_ENTRY_SHORT *pEntryP;
		RVLQLIST *parentRGBHOG;
		parentRGBHOG = (RVLQLIST*)((CRVL3DMeshObject*)this->parentMeshObject)->m_rgbHOG9;
		pEntryT = (RVLQLIST_HIST_ENTRY_SHORT*)(parentRGBHOG)->pFirst;
		pEntryP = pEntryT;
		
		for(int i = 0; i < hist_bins; i++)
		{
			if (objrgbHOG9[i] != 0)
			{
				RVLQLIST_HIST_ENTRY_SHORT *pHistEntry;// = new RVLQLIST_HIST_ENTRY_SHORT[1];
				RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST_HIST_ENTRY_SHORT, pHistEntry);
				pHistEntry->adr = i;
				pHistEntry->value = objrgbHOG9[i];
				RVLQLIST_ADD_ENTRY(this->m_rgbHOG9, pHistEntry);
				
				//Provjeravamo s parentom
				while (pEntryT->adr < i)
				{
					pEntryP = pEntryT;
					pEntryT = (RVLQLIST_HIST_ENTRY_SHORT*)pEntryT->pNext;
				}
				if (pEntryT->adr = i)
					pEntryT->value += objrgbHOG9[i];
				else if (pEntryT->adr > i)
				{
					RVLQLIST_HIST_ENTRY_SHORT *pHistNewEntry;// = new RVLQLIST_HIST_ENTRY_SHORT[1];
					RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST_HIST_ENTRY_SHORT, pHistNewEntry);
					pHistNewEntry->adr = i;
					pHistNewEntry->value = objrgbHOG9[i];
					RVLQLIST_INSERT_ENTRY(parentRGBHOG, pEntryP, pEntryT, pHistNewEntry);
				}

			}
		}				
	}
	else
	{
		for(int i = 0; i < hist_bins; i++)
		{
			if (objrgbHOG9[i] != 0)
			{
				RVLQLIST_HIST_ENTRY_SHORT *pHistEntry;// = new RVLQLIST_HIST_ENTRY_SHORT[1];
				RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST_HIST_ENTRY_SHORT, pHistEntry);
				pHistEntry->adr = i;
				pHistEntry->value = objrgbHOG9[i];
				RVLQLIST_ADD_ENTRY(this->m_rgbHOG9, pHistEntry);
			}
		}
	}

	//delete [] angle_r;
	//delete [] magnitude_r;
	//delete [] angle_g;
	//delete [] magnitude_g;
	//delete [] angle_b;
	//delete [] magnitude_b;
	//delete [] map;
	//delete [] rgbHOG9;
	//delete [] objrgbHOG9;
}

void CRVL3DMeshObject::RVLCalculateGrayHOG(IplImage* grayImg,
										  float *x,
										  float *y,
										  float *angle,
										  float *magnitude, 
										  bool *map,
										  float maxMagnitude,
										  int HOG_TYPE)
{
	//pomocne varijable
	int h = grayImg->height;
	int w = grayImg->width;
	/*float *angle;// = new float[h * w];
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem, float, h * w, angle)
	memset(angle, 0, h * w * sizeof(float));
	float *magnitude;// = new float[h * w];
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem, float, h * w, magnitude)
	memset(magnitude, 0, h * w * sizeof(float));
	bool *map;// = new bool[h * w];
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem, bool, h * w, map)
	memset(map, 0, h * w * sizeof(bool));*/
	float ang3x3[9];
	float mag3x3[9];
	float pixHOG[9];

	int adr = 0, adrD = 0;
	int orient;
	int orientMag;
	float binSizeAngle = PI / 9.0;
	float binSizeMagnitude = maxMagnitude / 10.0;
	float posAng;
	float posMag;
	int hist_bins;
	if (HOG_TYPE == RVL_MESH_HOG_1D)
		hist_bins = 9;
	else if (HOG_TYPE == RVL_MESH_HOG_2D)
		hist_bins = 9 * 10;
	else
		return;

	CRVL2DRegion2 *pTriangle;
	RVL3DPOINT2 **ppPt, **pPtArrayEnd;
	RVL3DPOINT2 *pPt;
	int iPix, iPixD;
	int label;
	ushort *HOG9;// = new ushort[hist_bins];
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, ushort, hist_bins, HOG9)
	ushort *objHOG9;// = new ushort[hist_bins];
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, ushort, hist_bins, objHOG9)
	memset(objHOG9, 0, hist_bins * sizeof(ushort));

	uchar *ptr;
	RVLQLIST_PTR_ENTRY *pElement;
	pElement = (RVLQLIST_PTR_ENTRY*)this->m_FaceList->pFirst;
	float br = 0.0;
	while(pElement)
	{
		pTriangle = (CRVL2DRegion2 *)(pElement->Ptr);

		if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		label = pTriangle->m_Label;

		pPtArrayEnd = pTriangle->m_pPoint3DArray + pTriangle->m_n3DPts;

		memset(HOG9, 0, hist_bins * sizeof(ushort));
		//pTriangle->m_rgbHOG9;// = new RVLQLIST[1];
		RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST, pTriangle->m_rgbHOG9);
		RVLQLIST_INIT(pTriangle->m_rgbHOG9);
		//br = 0.0;
		for(ppPt = pTriangle->m_pPoint3DArray; ppPt < pPtArrayEnd; ppPt++)
		{
			pPt = *ppPt;
			
			iPixD = pPt->iPix;
			iPix = pPt->iPixRGB;

			//br = 0.0;	//Nepotrebna varijabla ali zbog bizarnog buga s memorijom ocito potrebna...
			//ako je maska postavljena i nije dozvoljen idemo dalje
			if (pTriangle->m_uPixMask && (pTriangle->m_uPixMask[iPixD] == 1))
				continue;
			memset(pixHOG, 0.0, 9 * sizeof(float));
			//racunamo okolinu
			for (int i = 0; i < 9; i++)
			{
				//za koji piksel racunamo(hardcoded? na 320x240???)
				if ((i == 0) && (iPix != 0))
				{
					adr = iPix - grayImg->width - 1; //gornji lijevi
					adrD = iPixD - grayImg->width - 1;
				}
				else if ((i == 1) && (iPix > grayImg->width))
				{
					adr = iPix - grayImg->width; //gornji srednji
					adrD = iPixD - grayImg->width;
				}
				else if ((i == 2) && (iPix > grayImg->width) && ((iPix + 1) % grayImg->width != 0))
				{
					adr = iPix - grayImg->width + 1; //gornji desni
					adrD = iPixD - grayImg->width + 1;
				}
				else if ((i == 3) && (iPix != grayImg->width - 1))
				{
					adr = iPix - 1; //srednji lijevi
					adrD = iPixD - 1;
				}
				else if (i == 4)
				{
					adr = iPix; //srednji(promatrani)
					adrD = iPixD;
				}
				else if ((i == 5) && ((iPix + 1) % grayImg->width != 0))
				{
					adr = iPix + 1; //srednji desni
					adrD = iPixD + 1;
				}
				else if ((i == 6) && (iPix % grayImg->width != 0) && ((iPix + grayImg->width) < grayImg->width * grayImg->height))
				{
					adr = iPix + grayImg->width - 1; //donji lijevi
					adrD = iPixD + grayImg->width - 1;
				}
				else if ((i == 7) && ((iPix + grayImg->width) < grayImg->width * grayImg->height))
				{
					adr = iPix + grayImg->width; //donji srednji
					adrD = iPixD + grayImg->width;
				}
				else if ((i == 8) && ((iPix + grayImg->width) < grayImg->width * grayImg->height) && (iPix != (grayImg->width * grayImg->height - 1)))
				{
					adr = iPix + grayImg->width + 1; //donji desni
					adrD = iPixD + grayImg->width + 1;
				}
				else
				{
					ang3x3[i] = 0.0;
					mag3x3[i] = 0.0;
					continue;
				}
				//ako nije dozvoljen idemo dalje
				if (pTriangle->m_uPixMask && (pTriangle->m_uPixMask[adrD] == 1))
				{
					ang3x3[i] = 0.0;
					mag3x3[i] = 0.0;
					continue;
				}

				if (map[adr])
				{
					ang3x3[i] = angle[adr];
					mag3x3[i] = magnitude[adr];
					continue;
				}

				angle[adr] = atan2(y[adr], x[adr]);
				magnitude[adr] = sqrt(x[adr] * x[adr] + y[adr] * y[adr]);
				map[adr] = TRUE;
				ang3x3[i] = angle[adr];
				mag3x3[i] = magnitude[adr];
			}

			//Odredujemo kojem binu pripada orjentacija i stvaramo histogram
			for (int i = 0; i < 9; i++)
			{
				if (ang3x3[i] < 0)
					ang3x3[i] += PI;
				posAng = ang3x3[i] / binSizeAngle;				
				pixHOG[(int)posAng == 9 ? (int)(posAng - 1) : (int)posAng] += mag3x3[i];
			}
			br = Max(pixHOG, 9, orient);
			//povecavamo odgovarajuci bin histograma
			if (HOG_TYPE == RVL_MESH_HOG_1D)
			{
				//Gray
				HOG9[orient]++;
			}
			else if (HOG_TYPE == RVL_MESH_HOG_2D)
			{
				posMag = pixHOG[orient] / binSizeMagnitude;				
				if ((int)posMag >= 10)
					orientMag = 9;
				else
					orientMag = (int)posMag;
				//Gray
				HOG9[orientMag * 9 + orient]++;
			}
		}

		for(int i = 0; i < hist_bins; i++)
		{
			if (HOG9[i] != 0)
			{
				RVLQLIST_HIST_ENTRY_SHORT *pHistEntry;// = new RVLQLIST_HIST_ENTRY_SHORT[1];
				RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST_HIST_ENTRY_SHORT, pHistEntry);
				pHistEntry->adr = i;
				pHistEntry->value = HOG9[i];
				RVLQLIST_ADD_ENTRY(pTriangle->m_rgbHOG9, pHistEntry);
				objHOG9[i] += HOG9[i];
			}
		}

		pElement = (RVLQLIST_PTR_ENTRY*)pElement->pNext;
	}

	//ako postoji stari histogram, brisemo je (POTREBNO SAMO DOK SE NEPOVEZE SUSTAV S MEM strukturama)
	/*SUMNJIVO if (this->m_rgbHOG9->pFirst)
	{
		RVLQLIST_HIST_ENTRY_SHORT *pEntry = (RVLQLIST_HIST_ENTRY_SHORT*)this->m_rgbHOG9->pFirst;
		while(pEntry)
		{
			RVLQLIST_HIST_ENTRY_SHORT *pEntryD;
			pEntryD = pEntry;
			pEntry = new RVLQLIST_HIST_ENTRY_SHORT[1];
			pEntry = (RVLQLIST_HIST_ENTRY_SHORT*)pEntryD->pNext;
			delete pEntryD;
		}
		delete pEntry;
		RVLQLIST_INIT(this->m_rgbHOG9);
	}*/

	//Popunjavamo listu histograma i azuriramo roditeljski ako postoji
	if (((CRVL3DMeshObject*)this->parentMeshObject)->m_rgbHOG9)
	{
		RVLQLIST_HIST_ENTRY_SHORT *pEntryT;
		RVLQLIST_HIST_ENTRY_SHORT *pEntryP;
		RVLQLIST *parentRGBHOG;
		parentRGBHOG = (RVLQLIST*)((CRVL3DMeshObject*)this->parentMeshObject)->m_rgbHOG9;
		pEntryT = (RVLQLIST_HIST_ENTRY_SHORT*)(parentRGBHOG)->pFirst;
		pEntryP = pEntryT;
		
		for(int i = 0; i < hist_bins; i++)
		{
			if (objHOG9[i] != 0)
			{
				RVLQLIST_HIST_ENTRY_SHORT *pHistEntry;// = new RVLQLIST_HIST_ENTRY_SHORT[1];
				RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST_HIST_ENTRY_SHORT, pHistEntry);
				pHistEntry->adr = i;
				pHistEntry->value = objHOG9[i];
				RVLQLIST_ADD_ENTRY(this->m_rgbHOG9, pHistEntry);
				
				//Provjeravamo s parentom
				while (pEntryT->adr < i)
				{
					pEntryP = pEntryT;
					pEntryT = (RVLQLIST_HIST_ENTRY_SHORT*)pEntryT->pNext;
				}
				if (pEntryT->adr = i)
					pEntryT->value += objHOG9[i];
				else if (pEntryT->adr > i)
				{
					RVLQLIST_HIST_ENTRY_SHORT *pHistNewEntry;// = new RVLQLIST_HIST_ENTRY_SHORT[1];
					RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST_HIST_ENTRY_SHORT, pHistNewEntry);
					pHistNewEntry->adr = i;
					pHistNewEntry->value = objHOG9[i];
					RVLQLIST_INSERT_ENTRY(parentRGBHOG, pEntryP, pEntryT, pHistNewEntry);
				}

			}
		}				
	}
	else
	{
		for(int i = 0; i < hist_bins; i++)
		{
			if (objHOG9[i] != 0)
			{
				RVLQLIST_HIST_ENTRY_SHORT *pHistEntry;// = new RVLQLIST_HIST_ENTRY_SHORT[1];
				RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST_HIST_ENTRY_SHORT, pHistEntry);
				pHistEntry->adr = i;
				pHistEntry->value = objHOG9[i];
				RVLQLIST_ADD_ENTRY(this->m_rgbHOG9, pHistEntry);
			}
		}
	}

	//delete [] angle;
	//delete [] magnitude;
	//delete [] map;
	//delete [] HOG9;
	//delete [] objHOG9;
}

void CRVL3DMeshObject::RVLCalculateEllipsoidDescriptor(RVL3DPOINT2 **Point3DMap)
{
	RVLQLIST_PTR_ENTRY *pElement;
	pElement = (RVLQLIST_PTR_ENTRY*)this->m_FaceList->pFirst;
	CRVL2DRegion2 *pPolygon, *pPolygonNext;
	RVLMESH_LINK *pLink0, *pLink, *pLinkNext;
	int iPix;
	RVL3DPOINT2 *pPt;

	int noTriangles = this->m_noFaces;
	double xyz_centroid[3];
	memset(xyz_centroid, 0, 3 * sizeof(double));

	double *vertXYZ;// = new double[noTriangles * 3 * 3];
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, double, noTriangles * 3 * 3, vertXYZ)
	memset(vertXYZ, 0, noTriangles * 3 * 3 * sizeof(double));
	int brV = 0;
	while(pElement)
	{
		pPolygon = (CRVL2DRegion2 *)(pElement->Ptr);
		if(pPolygon->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;
		pLink = pLink0 = (RVLMESH_LINK *)(pPolygon->m_PtArray);
		do
		{
			iPix = pLink->iPix0;			
			pPolygonNext = (CRVL2DRegion2 *)(pLink->vp2DRegion);
			//
			xyz_centroid[0] += Point3DMap[iPix]->XYZ[0];
			xyz_centroid[1] += Point3DMap[iPix]->XYZ[1];
			xyz_centroid[2] += Point3DMap[iPix]->XYZ[2];
			vertXYZ[brV * 3] += Point3DMap[iPix]->XYZ[0];
			vertXYZ[brV * 3 + 1] += Point3DMap[iPix]->XYZ[1];
			vertXYZ[brV * 3 + 2] += Point3DMap[iPix]->XYZ[2];
			brV++;
			pLink = pLink->pNext->pOpposite;
		}while(pLink != pLink0);
		pElement = (RVLQLIST_PTR_ENTRY*)pElement->pNext;
	}
	xyz_centroid[0] /= brV;
	xyz_centroid[1] /= brV;
	xyz_centroid[2] /= brV;


	double demean[3], demean_xy, demean_xz, demean_yz;
	//double* C = new double[9];
	double C[9];
	memset(C, 0, 9 * sizeof(double));
	//double *eig = new double[3];
	double eig[3];
	memset(eig, 0, 3 * sizeof(double));
	//double *eigV = new double[9];
	double eigV[9];
	memset(eigV, 0, 9 * sizeof(double));
	for (int i = 0; i < brV; i++)
	{
		demean[0] = vertXYZ[i * 3] - xyz_centroid[0];
		demean[1] = vertXYZ[i * 3 + 1] - xyz_centroid[1];
		demean[2] = vertXYZ[i * 3 + 2] - xyz_centroid[2];
		demean_xy = demean[0] * demean[1];
		demean_xz = demean[0] * demean[2];
		demean_yz = demean[1] * demean[2];
		C[0] += demean[0] * demean[0];
		C[1] += demean_xy;
		C[2] += demean_xz;
		C[3] += demean_xy;
		C[4] += demean[1] * demean[1];
		C[5] += demean_yz;
		C[6] += demean_xz;
		C[7] += demean_yz;
		C[8] += demean[2] * demean[2];
	}
	C[0] /= brV;
	C[1] /= brV;
	C[2] /= brV;
	C[3] /= brV;
	C[4] /= brV;
	C[5] /= brV;
	C[6] /= brV;
	C[7] /= brV;
	C[8] /= brV;
	BOOL bReal[3];
	RVLEig3(C, eig, bReal);
	if (bReal[0] && bReal[1] && bReal[2])
	{
		if (eig[0] > 0)
			this->m_Ellipsoid[0] = sqrt(eig[0]);
		if (eig[1] > 0)
			this->m_Ellipsoid[1] = sqrt(eig[1]);
		if (eig[2] > 0)
			this->m_Ellipsoid[2] = sqrt(eig[2]);
	}
	//brisemo varijable
	//delete [] C;
	//delete [] eig;
	//delete [] eigV;
	//delete [] vertXYZ;
}

/*Za sada beskorisno void CRVL3DMeshObject::RVLCalculateD2ShapeDistribution(RVL3DPOINT2 **Point3DMap)
{
	RVLQLIST_PTR_ENTRY *pElement;
	pElement = (RVLQLIST_PTR_ENTRY*)this->m_FaceList->pFirst;
	CRVL2DRegion2 *pPolygon, *pPolygonNext;
	RVLMESH_LINK *pLink0, *pLink, *pLinkNext;
	int iPix;
	RVL3DPOINT2 *pPt;

	int noTriangles = this->m_noFaces;

	double *vertXYZ = new double[noTriangles * 3 * 3];
	memset(vertXYZ, 0, noTriangles * 3 * 3 * sizeof(double));
	double *dist = new double[(((noTriangles * 3) *  (noTriangles * 3)) / 2)];
	memset(dist, 0, (((noTriangles * 3) *  (noTriangles * 3)) / 2)  * sizeof(double));
	int brV = 0;
	while(pElement)
	{
		pPolygon = (CRVL2DRegion2 *)(pElement->Ptr);
		if(pPolygon->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;
		pLink = pLink0 = (RVLMESH_LINK *)(pPolygon->m_PtArray);
		do
		{
			iPix = pLink->iPix0;			
			pPolygonNext = (CRVL2DRegion2 *)(pLink->vp2DRegion);
			//racunamo X, Y, Z
			vertXYZ[brV * 3] += Point3DMap[iPix]->XYZ[0];
			vertXYZ[brV * 3 + 1] += Point3DMap[iPix]->XYZ[0];
			vertXYZ[brV * 3 + 2] += Point3DMap[iPix]->XYZ[0];
			brV++;
			pLink = pLink->pNext->pOpposite;
		}while(pLink != pLink0);
		pElement = (RVLQLIST_PTR_ENTRY*)pElement->pNext;
	}
	//racunamo udaljenosti
	double temp[3];
	int noLen = 0;
	for (int i = 0; i < brV; i++)
	{
		for (int j = 0; j < i; j++)
		{
			RVLDif3D(&vertXYZ[i * 3], &vertXYZ[j * 3], temp);
			dist[noLen] = RVLGet3DVectorLen(temp);
			noLen++;
		}
	}
	double maxLen;
	maxLen = Max(dist, noLen);
	double binSize = maxLen / 512;
	//pronalazimo distribuciju/histogram
	int hist[512];
	memset(hist, 0, 512 * sizeof(int));
	int pos = 0;
	for (int i = 0; i < noLen; i++)
	{
		pos = dist[i] / binSize;
		hist[(int)pos == 512 ? (int)(pos - 1) : (int)pos]++;
	}
	//ako postoji stara distribucija, brisemo je (POTREBNO SAMO DOK SE NEPOVEZE SUSTAV S MEM strukturama)
	/*SUMNJIVO if (this->m_shapeD2->pFirst)
	{
		RVLQLIST_DISTRIBUTION_ENTRY *pEntry = (RVLQLIST_DISTRIBUTION_ENTRY*)this->m_shapeD2->pFirst;
		while(pEntry)
		{
			RVLQLIST_DISTRIBUTION_ENTRY *pEntryD;
			pEntryD = pEntry;
			pEntry = new RVLQLIST_DISTRIBUTION_ENTRY[1];
			pEntry = (RVLQLIST_DISTRIBUTION_ENTRY*)pEntryD->pNext;
			delete pEntryD;
		}
		delete pEntry;
		RVLQLIST_INIT(this->m_shapeD2);
	}/

	RVLQLIST_DISTRIBUTION_ENTRY *pHistNULLEntry = new RVLQLIST_DISTRIBUTION_ENTRY[1];
	pHistNULLEntry->adr = -1;
	pHistNULLEntry->value = maxLen;
	RVLQLIST_ADD_ENTRY(this->m_shapeD2, pHistNULLEntry);
	maxLen = (double)Max(hist, 512);
	for (int i = 0; i < 512; i++)
	{
		if (hist[i] != 0)
		{
			RVLQLIST_DISTRIBUTION_ENTRY *pHistEntry = new RVLQLIST_DISTRIBUTION_ENTRY[1];
			pHistEntry->adr = i;
			pHistEntry->value = hist[i] / maxLen;
			RVLQLIST_ADD_ENTRY(this->m_shapeD2, pHistEntry);
		}
	}
	
	//brisemo varijable
	delete [] vertXYZ;
	delete [] dist;
}*/

void CRVL3DMeshObject::RVLCalculateTextureLabel()
{
	if (!this->m_rgbHOG9)
		return;
	//Works only with 2D HOG!!!
	//Hardcoded to 9 * 10 (GRAYSCALE)
	int hist_bins = 9 * 10;
	int *rgbHOG9;// = new int[hist_bins];
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, int, hist_bins, rgbHOG9);
	memset(rgbHOG9, 0, hist_bins * sizeof(int));
	//Transfering histogram from list to array
	RVLQLIST_HIST_ENTRY_SHORT *pHistEntry;
	pHistEntry = (RVLQLIST_HIST_ENTRY_SHORT*)this->m_rgbHOG9->pFirst;
	while(pHistEntry)
	{
		rgbHOG9[pHistEntry->adr] = pHistEntry->value;
		pHistEntry = (RVLQLIST_HIST_ENTRY_SHORT*)pHistEntry->pNext;
	}
	//calculating seperate scale and angle histograms
	int scaleHist[10];
	memset(scaleHist, 0, 10 * sizeof(int));
	int angleHist[9];
	memset(angleHist, 0, 9 * sizeof(int));
	int br = 0;
	int sumHOG = 0;
	for (int j = 0; j < 10; j++)
	{
		for (int i = 0; i < 9; i++)
		{
			scaleHist[j] += rgbHOG9[br];
			angleHist[i] += rgbHOG9[br];
			sumHOG += rgbHOG9[br];
			br++;
		}
	}
	//if there is no data in HOG, texture label is unknown
	if (sumHOG == 0)
	{
		this->m_TextureLabel = 0;	//unknown texture
		return;
	}
	//if more than 70% of hog energy is in first scale bin then texture is plain
	if ((float)scaleHist[0] / (float)sumHOG > 0.7)
	{
		this->m_TextureLabel = 1;	//plain texture
		return;
	}
	int tempPrim[2];
	memset(tempPrim, 0, 2 * sizeof(int));
	tempPrim[1] = Max(angleHist, 9, tempPrim[0]); 
	int *pPointsIdx;
	int n;
	pPointsIdx = RVLWhereAbove(angleHist, 9, (double)0.65*tempPrim[1], n);
	//Check if there is only one value above 0.65*max
	if (n==1 || (n==2 && pPointsIdx[0] == 0 && pPointsIdx[1] == 8))
	{
		this->m_TextureLabel = 2;
		return;
	}
	//Eliminating angleHists below threshold
	int mask[9];
	memset(mask, 0, 9 * sizeof(int));
	for (int i = 0; i < n; i++)
		mask[pPointsIdx[i]] = 1;
	for (int i = 0; i < 9; i++)
	{
		if (mask[i] == 0)
			angleHist[i] = 0;
	}
	int lab = 2;
	br = 0;
	while (br != n)
	{
		//eliminating surrounding of current max value
		for (int i = 0; i < n; i++)
		{
			if (pPointsIdx[i] != -1 && (abs(tempPrim[0] - pPointsIdx[i]) < 2 || abs(tempPrim[0] - pPointsIdx[i]) == 8))
			{
				angleHist[pPointsIdx[i]] = 0;
				pPointsIdx[i] = -1;
				br++;
			}
		}
		if (br != n)
		{
			memset(tempPrim, 0, 2 * sizeof(int));
			tempPrim[1] = Max(angleHist, 9, tempPrim[0]); 
			lab++;
		}
	}
	this->m_TextureLabel = lab;
	
	//delete [] rgbHOG9;
	delete [] pPointsIdx;
}

void CRVL3DMeshObject::RVLCalculateADSD(RVL3DPOINT2 **Point3DMap, float useRatio)
{
	RVLQLIST_PTR_ENTRY *pElement;
	pElement = (RVLQLIST_PTR_ENTRY*)this->m_FaceList->pFirst;
	CRVL2DRegion2 *pPolygon, *pPolygonNext;
	RVLMESH_LINK *pLink0, *pLink, *pLinkNext;
	int iPix;
	RVL3DPOINT2 *pPt;

	int noTriangles = this->m_noFaces;
	double *normal;// = new double[noTriangles * 3];
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, double, noTriangles * 3, normal)
	memset(normal, 0, noTriangles * 3 * sizeof(double));
	double *norm;// = new double[noTriangles];
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, double, noTriangles, norm)
	memset(norm, 0, noTriangles * sizeof(double));
	int br = 0;
	double D0[3], D1[3];
	//Calculating normals and its norms
	while(pElement)
	{
		pPolygon = (CRVL2DRegion2 *)(pElement->Ptr);
		if(pPolygon->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;
		
		pLink = pLink0 = (RVLMESH_LINK *)(pPolygon->m_PtArray);
		D0[0] = Point3DMap[pLink->pNext->pOpposite->iPix0]->XYZ[0] - Point3DMap[pLink->iPix0]->XYZ[0];
		D0[1] = Point3DMap[pLink->pNext->pOpposite->iPix0]->XYZ[1] - Point3DMap[pLink->iPix0]->XYZ[1];
		D0[2] = Point3DMap[pLink->pNext->pOpposite->iPix0]->XYZ[2] - Point3DMap[pLink->iPix0]->XYZ[2];
		D1[0] = Point3DMap[pLink->pOpposite->iPix0]->XYZ[0] - Point3DMap[pLink->iPix0]->XYZ[0];
		D1[1] = Point3DMap[pLink->pOpposite->iPix0]->XYZ[1] - Point3DMap[pLink->iPix0]->XYZ[1];
		D1[2] = Point3DMap[pLink->pOpposite->iPix0]->XYZ[2] - Point3DMap[pLink->iPix0]->XYZ[2];
		CrossProduct(D0, D1, &normal[br * 3]);
		norm[br] = sqrt(normal[br * 3] * normal[br * 3] + normal[br * 3 + 1] * normal[br * 3 + 1] + normal[br * 3 + 2] * normal[br * 3 + 2]);
		br++;

		pElement = (RVLQLIST_PTR_ENTRY*)pElement->pNext;
	}
	//SORTING
	int *sortIdx;// = new int[noTriangles];
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, int, noTriangles, sortIdx);
	Sort(norm, sortIdx, noTriangles);

	//Calculating angles and making histogram
	float cosFI = 0.0;
	float angle = 0.0;
	float binSize = PI / 9;
	for (int i = (int)(useRatio * noTriangles); i < noTriangles; i++)
	{
		if (norm[i] == 0)
			continue;
		for (int j = (int)(useRatio * noTriangles); j < i; j++)
		{
			if (norm[j] == 0)
				continue;
			cosFI = (normal[sortIdx[i] * 3] * normal[sortIdx[j] * 3] + normal[sortIdx[i] * 3 + 1] * normal[sortIdx[j] * 3 + 1] + normal[sortIdx[i] * 3 + 2] * normal[sortIdx[j] * 3 + 2]) / (norm[i] * norm[j]);
			if (cosFI > 1.0)
				cosFI = 1.0;
			angle = acos(cosFI);
			if (angle < 0)
				angle += PI;
			this->m_ADSD[(int)(angle / binSize)] += (norm[i] / 2) * (norm[j] / 2);			
		}
		this->m_ADSD[0] += (norm[i] / 2) * (norm[i] / 2);
	}
	//delete [] normal;
	//delete [] norm;
	//delete [] sortIdx;
}

void CRVL3DMeshObject::RVLCalculateShapeLabel()
{
	//Bad conditions
	if (this->m_noFaces < 4 || !this->m_ADSD || !this->m_Ellipsoid)
		return;
	if (this->m_Ellipsoid[0] == 0 && this->m_Ellipsoid[1] == 0 && this->m_Ellipsoid[2])
		return;

	float adsdSum = 0;
	for (int i = 0; i < 9; i++)
		adsdSum += this->m_ADSD[i];
	if (adsdSum == 0)
		return;

	float elipRatios[2];
	float adsdRatios[3];
	//calculating ellipsoid ratios
	elipRatios[0] = this->m_Ellipsoid[1] / (this->m_Ellipsoid[2] + 0.000000001);	//ensures not to divide by zero
	elipRatios[1] = this->m_Ellipsoid[1] / (this->m_Ellipsoid[0] + 0.000000001);
	//calculating ADSD ratios
	adsdRatios[0] = (this->m_ADSD[0] + this->m_ADSD[1]) / adsdSum;
	adsdRatios[1] = (this->m_ADSD[0] + this->m_ADSD[1] + this->m_ADSD[3] + this->m_ADSD[4]) / adsdSum;
	adsdRatios[2] = (this->m_ADSD[1] + this->m_ADSD[2] + this->m_ADSD[3]) / adsdSum;

	//Determing shape label
	if (adsdRatios[0] > 0.7 && adsdRatios[2] < 0.5)
		this->m_ShapeLabel = 1;		//Plane
	else if ((adsdRatios[1] / adsdRatios[0]) > 1.4 && adsdRatios[2] < 0.5)
		this->m_ShapeLabel = 2;		//Cuboid
	else if (adsdRatios[2] > 0.5)
	{
		if (elipRatios[0] > 1.5)
			this->m_ShapeLabel = 3;	//Cylinder
		else
			this->m_ShapeLabel = 4; //Sphere
	}
}

void CRVL3DMeshObject::SaveMeshObject2Ply(FILE *fpPly,
									RVLQLIST *pTriangleList,
									RVL3DPOINT2 **Point3DMap,
									DWORD Flags,
									int min3DPtsPerc,
									DWORD Mask)	
{
	char* vertices = new char[500000];
	memset(vertices, 0, 500000 * sizeof(char));
	char* indices = new char[500000];
	memset(indices, 0, 500000 * sizeof(char));
	//char* normals = new char[500000];
	//memset(normals, 0, 500000 * sizeof(char));
	char* temp = new char[200];
	memset(temp, 0, 200 * sizeof(char));
	int noV = 0;
	int noI = 0;

	RVLQLIST_PTR_ENTRY *pElement;
	pElement = (RVLQLIST_PTR_ENTRY*)pTriangleList->pFirst;

	int ImageSize = 320 * 240;	//HARDCODED?

	int *IdxMap = new int[ImageSize];

	memset(IdxMap, 0, ImageSize * sizeof(int));

	int iPt = 0;

	CRVL2DRegion2 *pPolygon;
	CRVL2DRegion2 *pHullTriangle;

	RVLMESH_LINK *pLink0, *pLink;
	int iPix;
	RVL3DPOINT2 *pPt;
	double k;
	int *N;
	double lenN2;
	double fN[3];

	while(pElement)
	{
		pPolygon = (CRVL2DRegion2 *)(pElement->Ptr);

		if(pPolygon->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		if((pPolygon->m_Flags & Mask) != Mask)
			continue;

		if((100 * pPolygon->m_n3DPts < min3DPtsPerc * pPolygon->m_nPts))
			continue;

		pLink = pLink0 = (RVLMESH_LINK *)(pPolygon->m_PtArray);
		
		sprintf(temp, "3\t");
		strcat(indices, temp);
		do
		{
			iPix = pLink->iPix0;

			if(IdxMap[iPix] == 0)
			{
				pPt = Point3DMap[iPix];

				if(Flags & RVLPSD_SAVE_MESH_FLAG_HULL)
				{
					pHullTriangle = (CRVL2DRegion2 *)(pLink->pData);

					sprintf(temp, "%.4f\t%.4f\t%.4f\n", pPt->XYZ[0], pPt->XYZ[1], pPt->XYZ[2]);
					strcat(vertices, temp);
					noV++;
				}
				else
				{
					sprintf(temp, "%.4f\t%.4f\t%.4f\n", pPt->XYZ[0], pPt->XYZ[1], pPt->XYZ[2]);
					strcat(vertices, temp);
					noV++;
				}

				IdxMap[iPix] = iPt;

				iPt++;
			}

			sprintf(temp, "%d\t", IdxMap[iPix]);
			strcat(indices, temp);

			pLink = pLink->pNext->pOpposite;
		}
		while(pLink != pLink0);

		sprintf(temp, "\n");
		strcat(indices, temp);
		noI++;
		//pHullTriangle = (CRVL2DRegion2 *)(pLink0->pData);
		//if ( pHullTriangle->m_lenN != 0)
		//	sprintf(temp, "%.4f\t%.4f\t%.4f\n", pHullTriangle->m_N[0] / pHullTriangle->m_lenN, pHullTriangle->m_N[1] / pHullTriangle->m_lenN, pHullTriangle->m_N[2] / pHullTriangle->m_lenN);
		//else
		//	sprintf(temp, "%0\t0\t0\n");
		//strcat(normals, temp);
		pElement = (RVLQLIST_PTR_ENTRY*)pElement->pNext;
	}
	fprintf(fpPly, "ply\nformat ascii 1.0\n");
	fprintf(fpPly, "pElement vertex %d\n", noV);
	fprintf(fpPly, "property float x\nproperty float y\nproperty float z\n");
	fprintf(fpPly, "pElement face %d\n", noI);
	fprintf(fpPly, "property list uchar int vertex_indices\n");
	//fprintf(fpPly, "pElement normal %d\n", noI);
	//fprintf(fpPly, "property float x\nproperty float y\nproperty float z\n");
	fprintf(fpPly, "end_header\n");
	fprintf(fpPly, "%s", vertices);
	fprintf(fpPly, "%s", indices);
	//fprintf(fpPly, "%s", normals);

	delete[] IdxMap;
	delete[] vertices;
	delete[] indices;
	//delete[] normals;
	delete[] temp;
}

void CRVL3DMeshObject::SaveMeshObject2Ply2(FILE *fpPly2,
									 RVLQLIST *pTriangleList,
									 RVL3DPOINT2 **Point3DMap,
									 DWORD Flags,
									 int min3DPtsPerc,
									 DWORD Mask)	
{
	char* vertices = new char[500000];
	memset(vertices, 0, 500000 * sizeof(char));
	char* indices = new char[500000];
	memset(indices, 0, 500000 * sizeof(char));
	//char* normals = new char[500000];
	//memset(normals, 0, 500000 * sizeof(char));
	char* temp = new char[200];
	memset(temp, 0, 200 * sizeof(char));
	int noV = 0;
	int noI = 0;

	RVLQLIST_PTR_ENTRY *pElement;
	pElement = (RVLQLIST_PTR_ENTRY*)pTriangleList->pFirst;

	int ImageSize = 320 * 240;	//HARDCODED??

	int *IdxMap = new int[ImageSize];

	memset(IdxMap, 0, ImageSize * sizeof(int));

	int iPt = 0;

	CRVL2DRegion2 *pPolygon;
	CRVL2DRegion2 *pHullTriangle;

	RVLMESH_LINK *pLink0, *pLink;
	int iPix;
	RVL3DPOINT2 *pPt;
	double k;
	int *N;
	double lenN2;
	double fN[3];

	while(pElement)
	{
		pPolygon = (CRVL2DRegion2 *)(pElement->Ptr);

		if(pPolygon->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		if((pPolygon->m_Flags & Mask) != Mask)
			continue;

		if((100 * pPolygon->m_n3DPts < min3DPtsPerc * pPolygon->m_nPts))
			continue;

		pLink = pLink0 = (RVLMESH_LINK *)(pPolygon->m_PtArray);
		
		sprintf(temp, "3\t");
		strcat(indices, temp);
		do
		{
			iPix = pLink->iPix0;

			if(IdxMap[iPix] == 0)
			{
				pPt = Point3DMap[iPix];

				if(Flags & RVLPSD_SAVE_MESH_FLAG_HULL)
				{
					pHullTriangle = (CRVL2DRegion2 *)(pLink->pData);

					sprintf(temp, "%.4f\t%.4f\t%.4f\n", pPt->XYZ[0], pPt->XYZ[1], pPt->XYZ[2]);
					strcat(vertices, temp);
					noV++;
				}
				else
				{
					sprintf(temp, "%.4f\t%.4f\t%.4f\n", pPt->XYZ[0], pPt->XYZ[1], pPt->XYZ[2]);
					strcat(vertices, temp);
					noV++;
				}

				IdxMap[iPix] = iPt;

				iPt++;
			}

			sprintf(temp, "%d\t", IdxMap[iPix]);
			strcat(indices, temp);

			pLink = pLink->pNext->pOpposite;
		}
		while(pLink != pLink0);

		sprintf(temp, "\n");
		strcat(indices, temp);
		noI++;
		//pHullTriangle = (CRVL2DRegion2 *)(pLink0->pData);
		//sprintf(temp, "%.4f\t%.4f\t%.4f\n", pHullTriangle->m_N[0], pHullTriangle->m_N[1], pHullTriangle->m_N[2]);
		//strcat(normals, temp);
		pElement = (RVLQLIST_PTR_ENTRY*)pElement->pNext;
	}
	fprintf(fpPly2, "%d\n", noV);
	fprintf(fpPly2, "%d\n", noI);
	fprintf(fpPly2, "%s", vertices);
	fprintf(fpPly2, "%s", indices);

	delete[] IdxMap;
	delete[] vertices;
	delete[] indices;
	delete[] temp;
}

void CRVL3DMeshObject::SaveMeshObject2OFF(FILE *fpOFF,
									RVLQLIST *pTriangleList,
									RVL3DPOINT2 **Point3DMap,
									DWORD Flags,
									int min3DPtsPerc,
									DWORD Mask)	
{
	char* vertices = new char[500000];
	memset(vertices, 0, 500000 * sizeof(char));
	char* indices = new char[500000];
	memset(indices, 0, 500000 * sizeof(char));
	//char* normals = new char[500000];
	//memset(normals, 0, 500000 * sizeof(char));
	char* temp = new char[200];
	memset(temp, 0, 200 * sizeof(char));
	int noV = 0;

	int noI = 0;

	RVLQLIST_PTR_ENTRY *pElement;
	pElement = (RVLQLIST_PTR_ENTRY*)pTriangleList->pFirst;

	int ImageSize = 320 * 240;	//HARDCODED?

	int *IdxMap = new int[ImageSize];

	memset(IdxMap, 0, ImageSize * sizeof(int));

	int iPt = 0;

	CRVL2DRegion2 *pPolygon;
	CRVL2DRegion2 *pHullTriangle;

	RVLMESH_LINK *pLink0, *pLink;
	int iPix;
	RVL3DPOINT2 *pPt;
	double k;
	int *N;
	double lenN2;
	double fN[3];
	while(pElement)
	{
		pPolygon = (CRVL2DRegion2 *)(pElement->Ptr);

		if(pPolygon->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		if((pPolygon->m_Flags & Mask) != Mask)
			continue;

		if((100 * pPolygon->m_n3DPts < min3DPtsPerc * pPolygon->m_nPts))
			continue;

		pLink = pLink0 = (RVLMESH_LINK *)(pPolygon->m_PtArray);
		
		sprintf(temp, "3 ");
		strcat(indices, temp);
		do
		{
			iPix = pLink->iPix0;

			if(IdxMap[iPix] == 0)
			{
				pPt = Point3DMap[iPix];

				if(Flags & RVLPSD_SAVE_MESH_FLAG_HULL)
				{
					pHullTriangle = (CRVL2DRegion2 *)(pLink->pData);

					sprintf(temp, "%.4f %.4f %.4f\n", pPt->XYZ[0], pPt->XYZ[1], pPt->XYZ[2]);
					strcat(vertices, temp);
					noV++;
				}
				else
				{
					sprintf(temp, "%.4f %.4f %.4f\n", pPt->XYZ[0], pPt->XYZ[1], pPt->XYZ[2]);
					strcat(vertices, temp);
					noV++;
				}

				IdxMap[iPix] = iPt;

				iPt++;
			}

			sprintf(temp, "%d ", IdxMap[iPix]);
			strcat(indices, temp);

			pLink = pLink->pNext->pOpposite;
		}
		while(pLink != pLink0);

		sprintf(temp, "\n");
		strcat(indices, temp);
		noI++;
		//pHullTriangle = (CRVL2DRegion2 *)(pLink0->pData);
		//sprintf(temp, "%.4f\t%.4f\t%.4f\n", pHullTriangle->m_N[0], pHullTriangle->m_N[1], pHullTriangle->m_N[2]);
		//strcat(normals, temp);
		pElement = (RVLQLIST_PTR_ENTRY*)pElement->pNext;
	}
	fprintf(fpOFF, "OFF\n", noV);
	fprintf(fpOFF, "%d %d %d\n", noV, noI, noV + noI - 2);
	fprintf(fpOFF, "%s", vertices);
	fprintf(fpOFF, "%s", indices);

	delete[] IdxMap;
	delete[] vertices;
	delete[] indices;
	delete[] temp;
}

void CRVL3DMeshObject::SaveMeshObject2OBJ(FILE *fpOBJ,
										  FILE *fpMTL,
										  char* mtlFileName,
										  RVL3DPOINT2 **Point3DMap,
										  int colorType,
										  char* texFileName,
										  DWORD Flags,
										  int min3DPtsPerc,
										  DWORD Mask)	
{
	char* vertices = new char[500000];
	memset(vertices, 0, 500000 * sizeof(char));
	char* indices = new char[500000];
	memset(indices, 0, 500000 * sizeof(char));
	char* mtllib = new char[50000];
	memset(mtllib, 0, 50000 * sizeof(char));
	char* verticesTex;
	if (colorType == 2)
	{
		verticesTex = new char[500000];
		memset(verticesTex, 0, 500000 * sizeof(char));
	}
	//char* normals = new char[500000];
	//memset(normals, 0, 500000 * sizeof(char));
	char* temp = new char[200];
	memset(temp, 0, 200 * sizeof(char));
	int noV = 0;
	int noI = 0;
	float matR = 0.0, matG = 0.0, matB = 0.0;

	RVLQLIST *pMeshObjList;
	RVLQLIST_PTR_ENTRY *pElement;
	RVLQLIST_PTR_ENTRY *pObject;
	CRVL3DMeshObject *tempObj;
	if (this->m_FaceList)
	{
		pMeshObjList = new RVLQLIST[1];
		RVLQLIST_INIT(pMeshObjList);
		RVLQLIST_PTR_ENTRY *pObj = new RVLQLIST_PTR_ENTRY[1];
		pObj->Ptr = this;
		RVLQLIST_ADD_ENTRY(pMeshObjList, pObj);
	}
	else if (this->m_ChildMeshObjects)
	{
		pMeshObjList = this->m_ChildMeshObjects;
	}

	int ImageSize = 320 * 240;	//HARDCODED?

	int *IdxMap = new int[ImageSize];
	memset(IdxMap, 0, ImageSize * sizeof(int));

	int iPt = 0;

	CRVL2DRegion2 *pPolygon;
	CRVL2DRegion2 *pHullTriangle;

	RVLMESH_LINK *pLink0, *pLink;
	int iPix;
	RVL3DPOINT2 *pPt;
	double k;
	int *N;
	double lenN2;
	double fN[3];
	
	int r,g,b;
	int rgb_x = 0, rgb_y = 0;

	pObject = (RVLQLIST_PTR_ENTRY*)pMeshObjList->pFirst;
	int br = 0;
	RVLQLIST_HIST_ENTRY *pHistEntry;
	RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST_HIST_ENTRY, pHistEntry);
	while (pObject)
	{
		tempObj = (CRVL3DMeshObject*)pObject->Ptr;

		pElement = (RVLQLIST_PTR_ENTRY*)tempObj->m_FaceList->pFirst;
		sprintf(temp, "g object%d\nusemtl material_%d\n", br, br);
		strcat(indices, temp);
		//generiranje novog materijala
		if (colorType == 0)
		{
			matR = RVLRandom(0.0, 1.0);
			matG = RVLRandom(0.0, 1.0);
			matB = RVLRandom(0.0, 1.0);

			sprintf(temp, "newmtl material_%d\nKd %.5f %.5f %.5f\nillum 0\n\n", br, matR, matG, matB);
			strcat(mtllib, temp);
		}
		else if((colorType == 1) && tempObj->m_histRGB)
		{
			pHistEntry = (RVLQLIST_HIST_ENTRY *)(tempObj->m_histRGB->pFirst);
			if (pHistEntry && (tempObj->m_ColorSystem == RVL_MESH_COLOR_RGB))
			{
				r = floor(pHistEntry->adr / (tempObj->m_histRGB_base[1] * tempObj->m_histRGB_base[2]));
				matR = (float)((r * (256.0 / tempObj->m_histRGB_base[0])) + (tempObj->m_histRGB_base[0]/2)) /255.0;
				g = floor((pHistEntry->adr / (tempObj->m_histRGB_base[1] * tempObj->m_histRGB_base[2]) - r) * tempObj->m_histRGB_base[1]);
				matG = (float)((g * (256.0 / tempObj->m_histRGB_base[1])) + (tempObj->m_histRGB_base[1]/2)) /255.0;
				b = floor(((pHistEntry->adr / (tempObj->m_histRGB_base[1] * tempObj->m_histRGB_base[2]) - r) * tempObj->m_histRGB_base[1] - g) * tempObj->m_histRGB_base[2]);
				matB = (float)((b * (256.0 / tempObj->m_histRGB_base[2])) + (tempObj->m_histRGB_base[2]/2)) /255.0;
			}
			else
			{
				matR = 1.0;
				matG = 1.0;
				matB = 1.0;
			}

			sprintf(temp, "newmtl material_%d\nKd %.5f %.5f %.5f\nillum 0\n\n", br, matR, matG, matB);
			strcat(mtllib, temp);
		}
		else if (colorType == 2)
		{
			sprintf(temp, "newmtl material_%d\nKa 1.0 1.0 1.0\nKd 1.0 1.0 1.0\nKs 0.0 0.0 0.0\nTr 1.0\nillum 1\nNs 0.0\nmap_Kd %s\n\n", br, texFileName);
			strcat(mtllib, temp);			
		}
		
		while(pElement)
		{
			pPolygon = (CRVL2DRegion2 *)(pElement->Ptr);

			if(pPolygon->m_Flags & RVLOBJ2_FLAG_REJECTED)
				continue;

			if((pPolygon->m_Flags & Mask) != Mask)
				continue;

			if((100 * pPolygon->m_n3DPts < min3DPtsPerc * pPolygon->m_nPts))
				continue;

			pLink = pLink0 = (RVLMESH_LINK *)(pPolygon->m_PtArray);
			
			sprintf(temp, "f ");
			strcat(indices, temp);
			do
			{
				iPix = pLink->iPix0;

				if(IdxMap[iPix] == 0)
				{
					pPt = Point3DMap[iPix];

					if(Flags & RVLPSD_SAVE_MESH_FLAG_HULL)
					{
						pHullTriangle = (CRVL2DRegion2 *)(pLink->pData);
						sprintf(temp, "v %.4f %.4f %.4f\n", pPt->XYZ[0], pPt->XYZ[1], pPt->XYZ[2]);
						strcat(vertices, temp);
						noV++;
					}
					else
					{
						sprintf(temp, "v %.4f %.4f %.4f\n", pPt->XYZ[0], pPt->XYZ[1], pPt->XYZ[2]);
						strcat(vertices, temp);
						noV++;
					}

					if (colorType == 2)
					{
						//hardcoded dimensions???
						rgb_y = pPt->iPixRGB / 320;
						rgb_x = pPt->iPixRGB - rgb_y * 320;
						if (rgb_y != 0)
							rgb_x --;
						rgb_y = 240 - rgb_y;

						sprintf(temp, "vt %.4f %.4f\n", (float)(rgb_x / 320.0), (float)(rgb_y / 240.0));
						strcat(verticesTex, temp);
					}

					IdxMap[iPix] = iPt + 1;

					iPt++;
				}

				sprintf(temp, "%d/%d ", IdxMap[iPix], IdxMap[iPix]);
				strcat(indices, temp);

				pLink = pLink->pNext->pOpposite;
			}
			while(pLink != pLink0);

			sprintf(temp, "\n");
			strcat(indices, temp);
			noI++;
			//pHullTriangle = (CRVL2DRegion2 *)(pLink0->pData);
			//sprintf(temp, "%.4f\t%.4f\t%.4f\n", pHullTriangle->m_N[0], pHullTriangle->m_N[1], pHullTriangle->m_N[2]);
			//strcat(normals, temp);
			pElement = (RVLQLIST_PTR_ENTRY*)pElement->pNext;
		}
		pObject = (RVLQLIST_PTR_ENTRY*)pObject->pNext;
		br++;
	}
	fprintf(fpOBJ, "mtllib ./%s\n", mtlFileName);
	fprintf(fpOBJ, "o scene1\n");
	fprintf(fpOBJ, "#vertices=%d faces=%d\n", noV, noI);
	fprintf(fpOBJ, "%s", vertices);
	if (colorType == 2)
		fprintf(fpOBJ, "%s", verticesTex);
	fprintf(fpOBJ, "%s", indices);
	fprintf(fpMTL, "%s", mtllib);

	delete[] IdxMap;
	delete[] vertices;
	delete[] indices;
	delete[] mtllib;
	delete[] temp;
	if (colorType == 2)
		delete[] verticesTex;
}

void CRVL3DMeshObject::AppendMeshObject2OBJ(int &iPt, 
											int &br, 
											CRVL3DPose *pPose, 
											FILE *fpOBJ,
											FILE *fpMTL,
											char* mtlFileName,
											RVL3DPOINT2 **Point3DMap,
											int colorType,
											char* texFileName,
											DWORD Flags,
											int min3DPtsPerc,
											DWORD Mask)	
{
	char* vertices = new char[500000];
	memset(vertices, 0, 500000 * sizeof(char));
	char* indices = new char[500000];
	memset(indices, 0, 500000 * sizeof(char));
	char* mtllib = new char[50000];
	memset(mtllib, 0, 50000 * sizeof(char));
	char* verticesTex;
	if (colorType == 2)
	{
		verticesTex = new char[500000];
		memset(verticesTex, 0, 500000 * sizeof(char));
	}
	//char* normals = new char[500000];
	//memset(normals, 0, 500000 * sizeof(char));
	char* temp = new char[200];
	memset(temp, 0, 200 * sizeof(char));
	int noV = 0;
	int noI = 0;
	float matR = 0.0, matG = 0.0, matB = 0.0;

	RVLQLIST *pMeshObjList;
	RVLQLIST_PTR_ENTRY *pElement;
	RVLQLIST_PTR_ENTRY *pObject;
	CRVL3DMeshObject *tempObj;
	if (this->m_FaceList)
	{
		pMeshObjList = new RVLQLIST[1];
		RVLQLIST_INIT(pMeshObjList);
		RVLQLIST_PTR_ENTRY *pObj = new RVLQLIST_PTR_ENTRY[1];
		pObj->Ptr = this;
		RVLQLIST_ADD_ENTRY(pMeshObjList, pObj);
	}
	else if (this->m_ChildMeshObjects)
	{
		pMeshObjList = this->m_ChildMeshObjects;
	}

	int ImageSize = 320 * 240;	//HARDCODED?

	int *IdxMap = new int[ImageSize];
	memset(IdxMap, 0, ImageSize * sizeof(int));

	

	CRVL2DRegion2 *pPolygon;
	CRVL2DRegion2 *pHullTriangle;

	RVLMESH_LINK *pLink0, *pLink;
	int iPix;
	RVL3DPOINT2 *pPt;
	double k;
	int *N;
	double lenN2;
	double fN[3];
	
	int r,g,b;
	int rgb_x = 0, rgb_y = 0;

	pObject = (RVLQLIST_PTR_ENTRY*)pMeshObjList->pFirst;
	//int br = 0;
	RVLQLIST_HIST_ENTRY *pHistEntry;
	RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST_HIST_ENTRY, pHistEntry);
	while (pObject)
	{
		tempObj = (CRVL3DMeshObject*)pObject->Ptr;

		pElement = (RVLQLIST_PTR_ENTRY*)tempObj->m_FaceList->pFirst;
		sprintf(temp, "g object%d\nusemtl material_%d\n", br, br);
		strcat(indices, temp);
		//generiranje novog materijala
		if (colorType == 0)
		{
			matR = RVLRandom(0.0, 1.0);
			matG = RVLRandom(0.0, 1.0);
			matB = RVLRandom(0.0, 1.0);

			sprintf(temp, "newmtl material_%d\nKd %.5f %.5f %.5f\nillum 0\n\n", br, matR, matG, matB);
			strcat(mtllib, temp);
		}
		else if((colorType == 1) && tempObj->m_histRGB)
		{
			pHistEntry = (RVLQLIST_HIST_ENTRY *)(tempObj->m_histRGB->pFirst);
			if (pHistEntry && (tempObj->m_ColorSystem == RVL_MESH_COLOR_RGB))
			{
				r = floor(pHistEntry->adr / (tempObj->m_histRGB_base[1] * tempObj->m_histRGB_base[2]));
				matR = (float)((r * (256.0 / tempObj->m_histRGB_base[0])) + (tempObj->m_histRGB_base[0]/2)) /255.0;
				g = floor((pHistEntry->adr / (tempObj->m_histRGB_base[1] * tempObj->m_histRGB_base[2]) - r) * tempObj->m_histRGB_base[1]);
				matG = (float)((g * (256.0 / tempObj->m_histRGB_base[1])) + (tempObj->m_histRGB_base[1]/2)) /255.0;
				b = floor(((pHistEntry->adr / (tempObj->m_histRGB_base[1] * tempObj->m_histRGB_base[2]) - r) * tempObj->m_histRGB_base[1] - g) * tempObj->m_histRGB_base[2]);
				matB = (float)((b * (256.0 / tempObj->m_histRGB_base[2])) + (tempObj->m_histRGB_base[2]/2)) /255.0;
			}
			else
			{
				matR = 1.0;
				matG = 1.0;
				matB = 1.0;
			}

			sprintf(temp, "newmtl material_%d\nKd %.5f %.5f %.5f\nillum 0\n\n", br, matR, matG, matB);
			strcat(mtllib, temp);
		}
		else if (colorType == 2)
		{
			sprintf(temp, "newmtl material_%d\nKa 1.0 1.0 1.0\nKd 1.0 1.0 1.0\nKs 0.0 0.0 0.0\nTr 1.0\nillum 1\nNs 0.0\nmap_Kd %s\n\n", br, texFileName);
			strcat(mtllib, temp);			
		}
		
		while(pElement)
		{
			pPolygon = (CRVL2DRegion2 *)(pElement->Ptr);

			if(pPolygon->m_Flags & RVLOBJ2_FLAG_REJECTED)
				continue;

			if((pPolygon->m_Flags & Mask) != Mask)
				continue;

			if((100 * pPolygon->m_n3DPts < min3DPtsPerc * pPolygon->m_nPts))
				continue;

			pLink = pLink0 = (RVLMESH_LINK *)(pPolygon->m_PtArray);
			
			sprintf(temp, "f ");
			strcat(indices, temp);
			do
			{
				iPix = pLink->iPix0;

				if(IdxMap[iPix] == 0)
				{
					pPt = Point3DMap[iPix];

					double pPtMatrix [3] = {pPt->XYZ[0], pPt->XYZ[1], pPt->XYZ[2]};
					double RotMult [3];
					double newpPt [3];
					RVLMULMX3X3VECT(pPose->m_Rot, pPtMatrix, RotMult)
					RVLSUM3VECTORS(pPose->m_X, RotMult, newpPt)

					if(Flags & RVLPSD_SAVE_MESH_FLAG_HULL)
					{
						pHullTriangle = (CRVL2DRegion2 *)(pLink->pData);
						sprintf(temp, "v %.4f %.4f %.4f\n", newpPt[0], newpPt[1], newpPt[2]);
						strcat(vertices, temp);
						noV++;
					}
					else
					{
						sprintf(temp, "v %.4f %.4f %.4f\n", newpPt[0], newpPt[1], newpPt[2]);
						strcat(vertices, temp);
						noV++;
					}

					if (colorType == 2)
					{
						//hardcoded dimensions???
						rgb_y = pPt->iPixRGB / 320;
						rgb_x = pPt->iPixRGB - rgb_y * 320;
						if (rgb_y != 0)
							rgb_x --;
						rgb_y = 240 - rgb_y;

						sprintf(temp, "vt %.4f %.4f\n", (float)(rgb_x / 320.0), (float)(rgb_y / 240.0));
						strcat(verticesTex, temp);
					}

					IdxMap[iPix] = iPt + 1;

					iPt++;
				}

				sprintf(temp, "%d/%d ", IdxMap[iPix], IdxMap[iPix]);
				strcat(indices, temp);

				pLink = pLink->pNext->pOpposite;
			}
			while(pLink != pLink0);

			sprintf(temp, "\n");
			strcat(indices, temp);
			noI++;
			//pHullTriangle = (CRVL2DRegion2 *)(pLink0->pData);
			//sprintf(temp, "%.4f\t%.4f\t%.4f\n", pHullTriangle->m_N[0], pHullTriangle->m_N[1], pHullTriangle->m_N[2]);
			//strcat(normals, temp);
			pElement = (RVLQLIST_PTR_ENTRY*)pElement->pNext;
		}
		pObject = (RVLQLIST_PTR_ENTRY*)pObject->pNext;
		br++;
	}
	fprintf(fpOBJ, "mtllib ./%s\n", mtlFileName);
	fprintf(fpOBJ, "o scene1\n");
	fprintf(fpOBJ, "#vertices=%d faces=%d\n", noV, noI);
	fprintf(fpOBJ, "%s", vertices);
	if (colorType == 2)
		fprintf(fpOBJ, "%s", verticesTex);
	fprintf(fpOBJ, "%s", indices);
	fprintf(fpMTL, "%s", mtllib);

	delete[] IdxMap;
	delete[] vertices;
	delete[] indices;
	delete[] mtllib;
	delete[] temp;
	if (colorType == 2)
		delete[] verticesTex;
}

//OLD BUT GOOD
//float CRVL3DMeshObject::RVLIntersectColorHistogram(CRVL3DMeshObject *model, int usedBins, bool hard, float *helperArray)
//{
//	//ako nepostoje odgovarajuci histogrami ili za nihovo stvaranje nije koristen ni jedan piksel u boji prekini
//	if (!this->m_histRGB || !model->m_histRGB || this->m_noUsedColorPts == 0 || model->m_noUsedColorPts == 0)
//		return 0.0;
//	//provjera da li su im baze histograma iste(NAPRAVITI DA SE REDUCIRA ONAJ HISTOGRAM KOJI JE PREVELIK!!!!)??????????
//	//takoder provjera da li su isti color sustavi koristeni
//	if ((this->m_histRGB_base[0] != model->m_histRGB_base[0]) || (this->m_histRGB_base[1] != model->m_histRGB_base[1]) || (this->m_histRGB_base[2] != model->m_histRGB_base[2]) || (this->m_ColorSystem != model->m_ColorSystem))
//		return -1.0;
//
//	RVLQLIST_HIST_ENTRY *pHistEntry;
//	int foundMaxObjBins = usedBins;
//	if (usedBins == -1)
//	{
//		float histbase[3] = {this->m_histRGB_base[0], this->m_histRGB_base[1], this->m_histRGB_base[2]};
//		if (histbase[0] == 0.0)	//if one of the dimensions is 0 it needs to be changed to 1 
//			histbase[0] = 1.0;
//		if (histbase[1] == 0.0)
//			histbase[1] = 1.0;
//		if (histbase[2] == 0.0)
//			histbase[2] = 1.0;
//		usedBins = histbase[0] * histbase[1] * histbase[2];
//		//pronalazimo koliko ima koristenih binova da bi smanjili zauzece memorije ako se koriste svi binovi
//		pHistEntry = (RVLQLIST_HIST_ENTRY*)this->m_histRGB->pFirst;
//		foundMaxObjBins = 0;
//		for (int i = 0; i < usedBins; i++)
//		{
//			foundMaxObjBins++;
//			pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
//			if (!pHistEntry)
//				break;
//		}
//		usedBins = foundMaxObjBins;	//Used bins becomes maximum found bins of current object
//	}
//	float intersectValue = 0.0;
//	float *objValues;// = new float[usedBins];
//	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, float, usedBins, objValues);
//
//	memset(objValues, 0, usedBins * sizeof(float));
//	int *objBinIdx;// = new int[usedBins];
//	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, int, usedBins, objBinIdx);
//	memset(objBinIdx, 0, usedBins * sizeof(int));
//	//float *modelValues = new float[usedBins];
//	//memset(modelValues, 0, usedBins * sizeof(float));
//	//int *modelBinIdx = new int[usedBins];
//	//memset(modelBinIdx, 0, usedBins * sizeof(int));
//	pHistEntry = (RVLQLIST_HIST_ENTRY*)this->m_histRGB->pFirst;
//	int foundObjBins = 0;
//	for (int i = 0; i < usedBins; i++)
//	{
//		objValues[i] = pHistEntry->value / (float)this->m_noUsedColorPts;
//		objBinIdx[i] = pHistEntry->adr;
//		foundObjBins++;
//		pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
//		if (!pHistEntry)
//			break;
//	}
//	int br = 0;
//	pHistEntry = (RVLQLIST_HIST_ENTRY*)model->m_histRGB->pFirst;
//	float modelValue = 0.0;
//	int i;
//	int foundObjBinsModel = 0;
//	while(pHistEntry)
//	{
//		if (hard && foundObjBinsModel >= usedBins)	//In case if we only want to look in top "usedBines"
//			break;
//
//		for (i = 0; i < foundObjBins; i++)
//		{
//			if (pHistEntry->adr == objBinIdx[i])
//			{
//				modelValue = pHistEntry->value / (float)model->m_noUsedColorPts;
//				if (modelValue < objValues[i])
//					intersectValue += modelValue;
//				else
//					intersectValue += objValues[i];
//				br++;
//				break;
//			}
//		}
//		if (br == foundObjBins)
//			break;
//		foundObjBinsModel++;
//		pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
//	}
//	//delete [] objValues;
//	//delete [] objBinIdx;
////#endif
//
////	// new code (CUPEC)
////
////#ifdef RVL_MESH_INTERSECT_COLOR_HISTOGRAM_DEBUG_LOG
////	FILE *fp;
////
////	fopen_s(&fp, "C:\\RVL\\Debug\\ColorHistogramM.dat", "w");
////#endif
////
////	float modelSelfIntersectValue = RVLCupecRGBProbMeasure(model, model, usedBins); //Self match, neede for relative match value
////
////	int *ColorData;
////
////	RVLMEM_ALLOC_STRUCT_ARRAY(model->m_pClass->m_pMem2, int, 3 * usedBins, ColorData);
////
////	int foundObjBinsModel = 0;
////	int *Cm = ColorData;
////	DWORD adr;
////	int base = (DOUBLE2INT(model->m_histRGB_base) >> 1);
////	int Shiftm = 0;
////
////	while(base)
////	{
////		Shiftm++;
////		base = (base >> 1);
////	}
////
////	double fnPts = 0.0;
////
////	int i;
////
////	RVLQLIST_HIST_ENTRY *pHistEntry = (RVLQLIST_HIST_ENTRY*)model->m_histRGB->pFirst;
////
////	for (i = 0; i < usedBins; i++)
////	{
////		if (!pHistEntry)
////			break;
////
////		fnPts += pHistEntry->value;
////
////		pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
////	}
////
////	pHistEntry = (RVLQLIST_HIST_ENTRY*)model->m_histRGB->pFirst;
////
////	for (i = 0; i < usedBins; i++)
////	{
////		objValues[i] = pHistEntry->value / fnPts;
////		adr = pHistEntry->adr;
////		*(Cm++) = ((adr & 0x00000007) << 1) + 1;
////		adr = (adr >> Shiftm);
////		*(Cm++) = ((adr & 0x00000007) << 1) + 1;
////		adr = (adr >> Shiftm);
////		*(Cm++) = ((adr & 0x00000007) << 1) + 1;
////		foundObjBinsModel++;
////		pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
////		if (!pHistEntry)
////			break;
////#ifdef RVL_MESH_INTERSECT_COLOR_HISTOGRAM_DEBUG_LOG
////		fprintf(fp, "%d\t%d\t%d\t%lf\n", ColorData[3 * i + 0], ColorData[3 * i + 1], ColorData[3 * i + 2], objValues[i]);
////#endif
////	}
////
////#ifdef RVL_MESH_INTERSECT_COLOR_HISTOGRAM_DEBUG_LOG
////		fclose(fp);
////
////		fopen_s(&fp, "C:\\RVL\\Debug\\ColorHistogramS.dat", "w");
////
////		FILE *fpTable;
////
////		fopen_s(&fpTable, "C:\\RVL\\Debug\\ColorMatchTable.dat", "w");
////#endif
////
////	// parameters
////
////	double sigman = 1.0 * 1.0;
////	double sigmas = 0.3 * 0.3;
////	double PClear = 0.9;
////
////	/////
////
////	double k1 = 2.0 * PI;
////	double k7 = k1 * k1 * k1;
////	double k2 = 2.0 * sigman * sigman;
////	double k3 = sigmas / sigman;
////	double k4 = 4.0 * k2 * sigmas;
////	double k5 = 4.0 * k2 * sigman;
////	double k6 = 1.0 / (2.0 * this->m_histRGB_base);
////	double k8 = (1.0 - PClear) * k6;
////	double k9 = k8 * k6 * k6;
////	
////	fnPts = 0.0;
////
////	int foundObjBins = 0;
////
////	pHistEntry = (RVLQLIST_HIST_ENTRY*)this->m_histRGB->pFirst;
////
////	while(pHistEntry)
////	{
////		if(foundObjBins >= usedBins)
////			break;
////
////		fnPts += pHistEntry->value;
////
////		foundObjBins++;
////
////		pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
////	}
////	
////	pHistEntry = (RVLQLIST_HIST_ENTRY*)this->m_histRGB->pFirst;
////
////	float w;
////	int C[3];
////	int tmp3x1[3], eC[3];
////	double det, lenCm2;
////	double pC, pCifCm;
////	double e1, e2, z1, z2;
////	int j;
////
////	for(i = 0; i < foundObjBins; i++)
////	{
////		w = 100.0 * pHistEntry->value / fnPts;
////		adr = pHistEntry->adr;
////		C[0] = ((adr & 0x00000007) << 1) + 1;
////		adr = (adr >> Shiftm);
////		C[1] = ((adr & 0x00000007) << 1) + 1;
////		adr = (adr >> Shiftm);
////		C[2] = ((adr & 0x00000007) << 1) + 1;
////
////#ifdef RVL_MESH_INTERSECT_COLOR_HISTOGRAM_DEBUG_LOG
////		fprintf(fp, "%d\t%d\t%d\t%lf\n", C[0], C[1], C[2], w);
////#endif
////
////		pC = k9;	// 3D PDF
////		//pC = k8;	// 1D PDF
////
////		for (j = 0; j < foundObjBinsModel; j++)
////		{
////			Cm = ColorData + 3 * j;
////
////			lenCm2 = (double)RVLDOTPRODUCT3(Cm, Cm);
////
////			det = k4 * lenCm2 + k5;  
////
////			RVLCROSSPRODUCT3(Cm, C, tmp3x1)
////			RVLDIF3VECTORS(C, Cm, eC)				// 3D PDF
////			//RVLSUM3VECTORS(C, Cm, eC)				// 1D PDF
////
////			e1 = (double)RVLDOTPRODUCT3(tmp3x1, tmp3x1);
////			e2 = (double)RVLDOTPRODUCT3(eC, eC);
////			
////			pCifCm = 1/sqrt(k7 * det) * exp(-k2/det*(k3*e1 + e2));	// 3D PDF
////			//pCifCm = 1/sqrt(k1 * sigman) * exp(-e1/e2/sigman);	// 1D PDF
////
////#ifdef RVL_MESH_INTERSECT_COLOR_HISTOGRAM_DEBUG_LOG
////			fprintf(fpTable, "%lf\t", pCifCm);
////#endif
////
////			pC += (pCifCm * objValues[j] * PClear);
////		}
////
////#ifdef RVL_MESH_INTERSECT_COLOR_HISTOGRAM_DEBUG_LOG
////		fprintf(fpTable, "\n");
////#endif
////		z1 = lnFactorial_LookUpTable[DOUBLE2INT(w)];	// 3D PDF, histogram probability
////		z2 = w * log(8.0 * pC);
////
////		intersectValue += (z1 - z2);	// histogram probability
////
////		//intersectValue -= z2;			// appearance probability
////
////		pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
////	}
////
////#ifdef RVL_MESH_INTERSECT_COLOR_HISTOGRAM_DEBUG_LOG
////	fclose(fp);
////
////	fclose(fpTable);
////#endif
////
////	/////
////
////	return abs(intersectValue - modelSelfIntersectValue);
//    this->m_pClass->m_pMem2->Clear();	//dodano radi smanjenja zauzeca memorije u slucaju statistickih analiza
//	return intersectValue;
//}

float CRVL3DMeshObject::RVLIntersectColorHistogram(CRVL3DMeshObject *model, int usedBins, bool hard, float *helperArray)
{
	//ako nepostoje odgovarajuci histogrami ili za nihovo stvaranje nije koristen ni jedan piksel u boji prekini
	if (!this->m_histRGB || !model->m_histRGB || this->m_noUsedColorPts == 0 || model->m_noUsedColorPts == 0)
		return 0.0;
	//provjera da li su im baze histograma iste(NAPRAVITI DA SE REDUCIRA ONAJ HISTOGRAM KOJI JE PREVELIK!!!!)??????????
	//takoder provjera da li su isti color sustavi koristeni
	if ((this->m_histRGB_base[0] != model->m_histRGB_base[0]) || (this->m_histRGB_base[1] != model->m_histRGB_base[1]) || (this->m_histRGB_base[2] != model->m_histRGB_base[2]) || (this->m_ColorSystem != model->m_ColorSystem))
		return -1.0;

	float intersectValue = 0.0;
	RVLQLIST_HIST_ENTRY *pHistEntry;
	if (usedBins == -1)
	{
		float histbase[3] = {this->m_histRGB_base[0], this->m_histRGB_base[1], this->m_histRGB_base[2]};
		if (histbase[0] == 0.0)	//if one of the dimensions is 0 it needs to be changed to 1 
			histbase[0] = 1.0;
		if (histbase[1] == 0.0)
			histbase[1] = 1.0;
		if (histbase[2] == 0.0)
			histbase[2] = 1.0;
		usedBins = histbase[0] * histbase[1] * histbase[2];
		memset(helperArray, 0, usedBins * sizeof(float));
		//pronalazimo koliko ima koristenih binova da bi smanjili zauzece memorije ako se koriste svi binovi
		pHistEntry = (RVLQLIST_HIST_ENTRY*)this->m_histRGB->pFirst;
		for (int i = 0; i < usedBins; i++)
		{
			helperArray[pHistEntry->adr] = pHistEntry->value / (float)this->m_noUsedColorPts;
			pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
			if (!pHistEntry)
				break;
		}

		pHistEntry = (RVLQLIST_HIST_ENTRY*)model->m_histRGB->pFirst;
		float modelValue = 0.0;
		while(pHistEntry)
		{
			if (helperArray[pHistEntry->adr] != 0.0)
			{
				modelValue = pHistEntry->value / (float)model->m_noUsedColorPts;
				if (modelValue < helperArray[pHistEntry->adr])
					intersectValue += modelValue;
				else
					intersectValue += helperArray[pHistEntry->adr];
			}
			pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
		}
	}
	else
	{
		float *objValues;// = new float[usedBins];
		RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, float, usedBins, objValues);

		memset(objValues, 0, usedBins * sizeof(float));
		int *objBinIdx;// = new int[usedBins];
		RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, int, usedBins, objBinIdx);
		memset(objBinIdx, 0, usedBins * sizeof(int));
		//float *modelValues = new float[usedBins];
		//memset(modelValues, 0, usedBins * sizeof(float));
		//int *modelBinIdx = new int[usedBins];
		//memset(modelBinIdx, 0, usedBins * sizeof(int));
		pHistEntry = (RVLQLIST_HIST_ENTRY*)this->m_histRGB->pFirst;
		int foundObjBins = 0;
		for (int i = 0; i < usedBins; i++)
		{
			objValues[i] = pHistEntry->value / (float)this->m_noUsedColorPts;
			objBinIdx[i] = pHistEntry->adr;
			foundObjBins++;
			pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
			if (!pHistEntry)
				break;
		}
		int br = 0;
		pHistEntry = (RVLQLIST_HIST_ENTRY*)model->m_histRGB->pFirst;
		float modelValue = 0.0;
		int i;
		int foundObjBinsModel = 0;
		while(pHistEntry)
		{
			if (hard && foundObjBinsModel >= usedBins)	//In case if we only want to look in top "usedBines"
				break;

			for (i = 0; i < foundObjBins; i++)
			{
				if (pHistEntry->adr == objBinIdx[i])
				{
					modelValue = pHistEntry->value / (float)model->m_noUsedColorPts;
					if (modelValue < objValues[i])
						intersectValue += modelValue;
					else
						intersectValue += objValues[i];
					br++;
					break;
				}
			}
			if (br == foundObjBins)
				break;
			foundObjBinsModel++;
			pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
		}
	}
	//delete [] objValues;
	//delete [] objBinIdx;
//#endif

//	// new code (CUPEC)
//
//#ifdef RVL_MESH_INTERSECT_COLOR_HISTOGRAM_DEBUG_LOG
//	FILE *fp;
//
//	fopen_s(&fp, "C:\\RVL\\Debug\\ColorHistogramM.dat", "w");
//#endif
//
//	float modelSelfIntersectValue = RVLCupecRGBProbMeasure(model, model, usedBins); //Self match, neede for relative match value
//
//	int *ColorData;
//
//	RVLMEM_ALLOC_STRUCT_ARRAY(model->m_pClass->m_pMem2, int, 3 * usedBins, ColorData);
//
//	int foundObjBinsModel = 0;
//	int *Cm = ColorData;
//	DWORD adr;
//	int base = (DOUBLE2INT(model->m_histRGB_base) >> 1);
//	int Shiftm = 0;
//
//	while(base)
//	{
//		Shiftm++;
//		base = (base >> 1);
//	}
//
//	double fnPts = 0.0;
//
//	int i;
//
//	RVLQLIST_HIST_ENTRY *pHistEntry = (RVLQLIST_HIST_ENTRY*)model->m_histRGB->pFirst;
//
//	for (i = 0; i < usedBins; i++)
//	{
//		if (!pHistEntry)
//			break;
//
//		fnPts += pHistEntry->value;
//
//		pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
//	}
//
//	pHistEntry = (RVLQLIST_HIST_ENTRY*)model->m_histRGB->pFirst;
//
//	for (i = 0; i < usedBins; i++)
//	{
//		objValues[i] = pHistEntry->value / fnPts;
//		adr = pHistEntry->adr;
//		*(Cm++) = ((adr & 0x00000007) << 1) + 1;
//		adr = (adr >> Shiftm);
//		*(Cm++) = ((adr & 0x00000007) << 1) + 1;
//		adr = (adr >> Shiftm);
//		*(Cm++) = ((adr & 0x00000007) << 1) + 1;
//		foundObjBinsModel++;
//		pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
//		if (!pHistEntry)
//			break;
//#ifdef RVL_MESH_INTERSECT_COLOR_HISTOGRAM_DEBUG_LOG
//		fprintf(fp, "%d\t%d\t%d\t%lf\n", ColorData[3 * i + 0], ColorData[3 * i + 1], ColorData[3 * i + 2], objValues[i]);
//#endif
//	}
//
//#ifdef RVL_MESH_INTERSECT_COLOR_HISTOGRAM_DEBUG_LOG
//		fclose(fp);
//
//		fopen_s(&fp, "C:\\RVL\\Debug\\ColorHistogramS.dat", "w");
//
//		FILE *fpTable;
//
//		fopen_s(&fpTable, "C:\\RVL\\Debug\\ColorMatchTable.dat", "w");
//#endif
//
//	// parameters
//
//	double sigman = 1.0 * 1.0;
//	double sigmas = 0.3 * 0.3;
//	double PClear = 0.9;
//
//	/////
//
//	double k1 = 2.0 * PI;
//	double k7 = k1 * k1 * k1;
//	double k2 = 2.0 * sigman * sigman;
//	double k3 = sigmas / sigman;
//	double k4 = 4.0 * k2 * sigmas;
//	double k5 = 4.0 * k2 * sigman;
//	double k6 = 1.0 / (2.0 * this->m_histRGB_base);
//	double k8 = (1.0 - PClear) * k6;
//	double k9 = k8 * k6 * k6;
//	
//	fnPts = 0.0;
//
//	int foundObjBins = 0;
//
//	pHistEntry = (RVLQLIST_HIST_ENTRY*)this->m_histRGB->pFirst;
//
//	while(pHistEntry)
//	{
//		if(foundObjBins >= usedBins)
//			break;
//
//		fnPts += pHistEntry->value;
//
//		foundObjBins++;
//
//		pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
//	}
//	
//	pHistEntry = (RVLQLIST_HIST_ENTRY*)this->m_histRGB->pFirst;
//
//	float w;
//	int C[3];
//	int tmp3x1[3], eC[3];
//	double det, lenCm2;
//	double pC, pCifCm;
//	double e1, e2, z1, z2;
//	int j;
//
//	for(i = 0; i < foundObjBins; i++)
//	{
//		w = 100.0 * pHistEntry->value / fnPts;
//		adr = pHistEntry->adr;
//		C[0] = ((adr & 0x00000007) << 1) + 1;
//		adr = (adr >> Shiftm);
//		C[1] = ((adr & 0x00000007) << 1) + 1;
//		adr = (adr >> Shiftm);
//		C[2] = ((adr & 0x00000007) << 1) + 1;
//
//#ifdef RVL_MESH_INTERSECT_COLOR_HISTOGRAM_DEBUG_LOG
//		fprintf(fp, "%d\t%d\t%d\t%lf\n", C[0], C[1], C[2], w);
//#endif
//
//		pC = k9;	// 3D PDF
//		//pC = k8;	// 1D PDF
//
//		for (j = 0; j < foundObjBinsModel; j++)
//		{
//			Cm = ColorData + 3 * j;
//
//			lenCm2 = (double)RVLDOTPRODUCT3(Cm, Cm);
//
//			det = k4 * lenCm2 + k5;  
//
//			RVLCROSSPRODUCT3(Cm, C, tmp3x1)
//			RVLDIF3VECTORS(C, Cm, eC)				// 3D PDF
//			//RVLSUM3VECTORS(C, Cm, eC)				// 1D PDF
//
//			e1 = (double)RVLDOTPRODUCT3(tmp3x1, tmp3x1);
//			e2 = (double)RVLDOTPRODUCT3(eC, eC);
//			
//			pCifCm = 1/sqrt(k7 * det) * exp(-k2/det*(k3*e1 + e2));	// 3D PDF
//			//pCifCm = 1/sqrt(k1 * sigman) * exp(-e1/e2/sigman);	// 1D PDF
//
//#ifdef RVL_MESH_INTERSECT_COLOR_HISTOGRAM_DEBUG_LOG
//			fprintf(fpTable, "%lf\t", pCifCm);
//#endif
//
//			pC += (pCifCm * objValues[j] * PClear);
//		}
//
//#ifdef RVL_MESH_INTERSECT_COLOR_HISTOGRAM_DEBUG_LOG
//		fprintf(fpTable, "\n");
//#endif
//		z1 = lnFactorial_LookUpTable[DOUBLE2INT(w)];	// 3D PDF, histogram probability
//		z2 = w * log(8.0 * pC);
//
//		intersectValue += (z1 - z2);	// histogram probability
//
//		//intersectValue -= z2;			// appearance probability
//
//		pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
//	}
//
//#ifdef RVL_MESH_INTERSECT_COLOR_HISTOGRAM_DEBUG_LOG
//	fclose(fp);
//
//	fclose(fpTable);
//#endif
//
//	/////
//
//	return abs(intersectValue - modelSelfIntersectValue);
    //this->m_pClass->m_pMem2->Clear();	//dodano radi smanjenja zauzeca memorije u slucaju statistickih analiza
	return intersectValue;
}

void CRVL3DMeshObject::RVLIncreaseRGBHistBinSizeSYM(float ratio)
{
	float newBase[3];
	newBase[0] = this->m_histRGB_base[0] / ratio;
	newBase[1] = this->m_histRGB_base[1] / ratio;
	newBase[2] = this->m_histRGB_base[2] / ratio;
	float x,y,z,xN,yN,zN;
	int adrN;
	RVLQLIST *newRGBHist;// = new RVLQLIST[1];
	RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST, newRGBHist);
	RVLQLIST_INIT(newRGBHist);

	RVLQLIST_HIST_ENTRY_SHORT *pOldHistEntry;
	pOldHistEntry = (RVLQLIST_HIST_ENTRY_SHORT*)this->m_histRGB->pFirst;
	RVLQLIST_HIST_ENTRY_SHORT *pNewHistEntry;
	while(pOldHistEntry)
	{
		//generating new address
		x = floor(pOldHistEntry->adr / (this->m_histRGB_base[1] * this->m_histRGB_base[2]));
		y = floor((pOldHistEntry->adr / (this->m_histRGB_base[1] * this->m_histRGB_base[2]) - x) * this->m_histRGB_base[1]);
		z = floor(((pOldHistEntry->adr / (this->m_histRGB_base[1] * this->m_histRGB_base[2]) - x) * this->m_histRGB_base[1] - y) * this->m_histRGB_base[2]);
		xN = floor(x / ratio);
		yN = floor(y / ratio);
		zN = floor(z / ratio);
		adrN = (int)(xN * newBase[1] * newBase[2] + yN * newBase[2] + zN);
		if (!newRGBHist->pFirst)
		{
			RVLQLIST_HIST_ENTRY_SHORT *pHistEntry = new RVLQLIST_HIST_ENTRY_SHORT[1];
			RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST_HIST_ENTRY_SHORT, pHistEntry);
			pHistEntry->adr = adrN;
			pHistEntry->value = pOldHistEntry->value;
			RVLQLIST_ADD_ENTRY(newRGBHist, pHistEntry);
		}
		else
		{
			pNewHistEntry = (RVLQLIST_HIST_ENTRY_SHORT*)newRGBHist->pFirst;
			while(pNewHistEntry)
			{
				if (pNewHistEntry->adr == adrN)
				{
					pNewHistEntry->value += pOldHistEntry->value;
					break;
				}
				pNewHistEntry = (RVLQLIST_HIST_ENTRY_SHORT*)pNewHistEntry->pNext;
				if (!pNewHistEntry)
				{
					RVLQLIST_HIST_ENTRY_SHORT *pHistEntry;// = new RVLQLIST_HIST_ENTRY_SHORT[1];
					RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST_HIST_ENTRY_SHORT, pHistEntry);
					pHistEntry->adr = adrN;
					pHistEntry->value = pOldHistEntry->value;
					RVLQLIST_ADD_ENTRY(newRGBHist, pHistEntry);
				}
			}
		}
		pOldHistEntry = (RVLQLIST_HIST_ENTRY_SHORT*)pOldHistEntry->pNext;
	}
	this->m_histRGB = newRGBHist;
	this->m_histRGB_base[0] = newBase[0];
	this->m_histRGB_base[1] = newBase[1];
	this->m_histRGB_base[2] = newBase[2];
}
RVLQLIST* CRVL3DMeshObject::RVLIncreaseRGBHistBinSizeSYM(RVLQLIST* histRGB, float* oldHistBase, float ratio)
{
	float newBase[3];
	newBase[0] = oldHistBase[0] / ratio;
	float x,y,z,xN,yN,zN;
	int adrN;
	RVLQLIST *newRGBHist = new RVLQLIST[1];
	//RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST, newRGBHist);	//Because of 'static' function we must have outside source of mem?!?!
	RVLQLIST_INIT(newRGBHist);

	RVLQLIST_HIST_ENTRY_SHORT *pOldHistEntry;
	pOldHistEntry = (RVLQLIST_HIST_ENTRY_SHORT*)histRGB->pFirst;
	RVLQLIST_HIST_ENTRY_SHORT *pNewHistEntry;
	while(pOldHistEntry)
	{
		//generating new address
		x = floor(pOldHistEntry->adr / (oldHistBase[1] * oldHistBase[2]));
		y = floor((pOldHistEntry->adr / (oldHistBase[1] * oldHistBase[2]) - x) * oldHistBase[1]);
		z = floor(((pOldHistEntry->adr / (oldHistBase[1] * oldHistBase[2]) - x) * oldHistBase[1] - y) * oldHistBase[2]);
		xN = floor(x / ratio);
		yN = floor(y / ratio);
		zN = floor(z / ratio);
		adrN = (int)(xN * newBase[1] * newBase[2] + yN * newBase[2] + zN);
		if (!newRGBHist->pFirst)
		{
			RVLQLIST_HIST_ENTRY_SHORT *pHistEntry = new RVLQLIST_HIST_ENTRY_SHORT[1];
			//RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST_HIST_ENTRY_SHORT, pHistEntry);
			pHistEntry->adr = adrN;
			pHistEntry->value = pOldHistEntry->value;
			RVLQLIST_ADD_ENTRY(newRGBHist, pHistEntry);
		}
		else
		{
			pNewHistEntry = (RVLQLIST_HIST_ENTRY_SHORT*)newRGBHist->pFirst;
			while(pNewHistEntry)
			{
				if (pNewHistEntry->adr == adrN)
				{
					pNewHistEntry->value += pOldHistEntry->value;
					break;
				}
				pNewHistEntry = (RVLQLIST_HIST_ENTRY_SHORT*)pNewHistEntry->pNext;
				if (!pNewHistEntry)
				{
					RVLQLIST_HIST_ENTRY_SHORT *pHistEntry = new RVLQLIST_HIST_ENTRY_SHORT[1];
					//RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST_HIST_ENTRY_SHORT, pHistEntry);
					pHistEntry->adr = adrN;
					pHistEntry->value = pOldHistEntry->value;
					RVLQLIST_ADD_ENTRY(newRGBHist, pHistEntry);
				}
			}
		}
		pOldHistEntry = (RVLQLIST_HIST_ENTRY_SHORT*)pOldHistEntry->pNext;
	}
	return newRGBHist;
}

void CRVL3DMeshObject::Save(	FILE *fp,
				DWORD Flags)
{
	int noBinsUsed = 0;
	RVLQLIST_HIST_ENTRY *pHistEntry;

	//Deskriptor boje
	pHistEntry = (RVLQLIST_HIST_ENTRY*)this->m_histRGB->pFirst;
	while(pHistEntry)
	{
		noBinsUsed++;
		pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
	}
	fwrite(&this->m_noUsedColorPts, sizeof(int), 1, fp);
	fwrite(&this->m_histRGB_base[0], sizeof(float), 1, fp);
	fwrite(&this->m_histRGB_base[1], sizeof(float), 1, fp);
	fwrite(&this->m_histRGB_base[2], sizeof(float), 1, fp);
	fwrite(&this->m_ColorSystem, sizeof(int), 1, fp);
	fwrite(&noBinsUsed, sizeof(int), 1, fp);
	pHistEntry = (RVLQLIST_HIST_ENTRY*)this->m_histRGB->pFirst;
	while(pHistEntry)
	{
		fwrite(&(*pHistEntry), sizeof(RVLQLIST_HIST_ENTRY), 1, fp);
		pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
	}

	////deskriptor teksture(LBP_RIU_VAR)
	//fwrite(&this->m_lbpP, sizeof(int), 1, fp);
	//fwrite(&this->m_lbpR, sizeof(int), 1, fp);
	//fwrite(&this->m_noUsedLbpPts, sizeof(int), 1, fp);
	////histogram
	//noBinsUsed = 0;
	//pHistEntry = (RVLQLIST_HIST_ENTRY*)this->m_LBP_RIU_VAR->pFirst;
	//while(pHistEntry)
	//{
	//	noBinsUsed++;
	//	pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
	//}
	//fwrite(&noBinsUsed, sizeof(int), 1, fp);
	//pHistEntry = (RVLQLIST_HIST_ENTRY*)this->m_LBP_RIU_VAR->pFirst;
	//while(pHistEntry)
	//{
	//	fwrite(&(*pHistEntry), sizeof(RVLQLIST_HIST_ENTRY), 1, fp);
	//	pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
	//}

	////deskriptor teksture(LBP_RIU)
	//fwrite(&this->m_lbpP, sizeof(int), 1, fp);
	//fwrite(&this->m_lbpR, sizeof(int), 1, fp);
	//fwrite(&this->m_noUsedLbpPts, sizeof(int), 1, fp);
	////histogram
	//noBinsUsed = 0;
	//pHistEntry = (RVLQLIST_HIST_ENTRY*)this->m_LBP_RIU->pFirst;
	//while(pHistEntry)
	//{
	//	noBinsUsed++;
	//	pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
	//}
	//fwrite(&noBinsUsed, sizeof(int), 1, fp);
	//pHistEntry = (RVLQLIST_HIST_ENTRY*)this->m_LBP_RIU->pFirst;
	//while(pHistEntry)
	//{
	//	fwrite(&(*pHistEntry), sizeof(RVLQLIST_HIST_ENTRY), 1, fp);
	//	pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
	//}


	////deskriptor teksture(LBP)
	//fwrite(&this->m_lbpP, sizeof(int), 1, fp);
	//fwrite(&this->m_lbpR, sizeof(int), 1, fp);
	//fwrite(&this->m_noUsedLbpPts, sizeof(int), 1, fp);
	////histogram
	//noBinsUsed = 0;
	//pHistEntry = (RVLQLIST_HIST_ENTRY*)this->m_LBP->pFirst;
	//while(pHistEntry)
	//{
	//	noBinsUsed++;
	//	pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
	//}
	//fwrite(&noBinsUsed, sizeof(int), 1, fp);
	//pHistEntry = (RVLQLIST_HIST_ENTRY*)this->m_LBP->pFirst;
	//while(pHistEntry)
	//{
	//	fwrite(&(*pHistEntry), sizeof(RVLQLIST_HIST_ENTRY), 1, fp);
	//	pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
	//}


	//fwrite(&this->m_TextureLabel, sizeof(int), 1, fp);
}
void CRVL3DMeshObject::Load(	FILE *fp,
				DWORD Flags)
{
	int noBinsUsed = 0;
	//color descriptor
	fread(&this->m_noUsedColorPts, sizeof(int), 1, fp);
	fread(&this->m_histRGB_base[0], sizeof(float), 1, fp);
	fread(&this->m_histRGB_base[1], sizeof(float), 1, fp);
	fread(&this->m_histRGB_base[2], sizeof(float), 1, fp);
	fread(&this->m_ColorSystem, sizeof(int), 1, fp);
	fread(&noBinsUsed, sizeof(int), 1, fp);
	
	RVLQLIST_HIST_ENTRY *pHistEntry;

	//Added by Karlo (initialize m_histRGB
	RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem0, RVLQLIST, this->m_histRGB);
	RVLQLIST_INIT(this->m_histRGB);
	
	for (int i = 0; i < noBinsUsed; i++)
	{
		RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem0, RVLQLIST_HIST_ENTRY, pHistEntry);  //added by Karlo 
		fread(pHistEntry, sizeof(RVLQLIST_HIST_ENTRY), 1, fp);
		RVLQLIST_ADD_ENTRY(this->m_histRGB, pHistEntry);
	}

	//fread(&this->m_ShapeLabel, sizeof(int), 1, fp);

	////texture descriptor LBP RIU VAR
	//fread(&this->m_lbpP, sizeof(int), 1, fp);
	//fread(&this->m_lbpR, sizeof(int), 1, fp);
	//fread(&this->m_noUsedLbpPts, sizeof(int), 1, fp);
	//fread(&noBinsUsed, sizeof(int), 1, fp);

	//RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem0, RVLQLIST, this->m_LBP_RIU_VAR);
	//RVLQLIST_INIT(this->m_LBP_RIU_VAR);

	//for (int i = 0; i < noBinsUsed; i++)
	//{
	//	RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem0, RVLQLIST_HIST_ENTRY, pHistEntry);  //added by Karlo 
	//	fread(pHistEntry, sizeof(RVLQLIST_HIST_ENTRY), 1, fp);
	//	RVLQLIST_ADD_ENTRY(this->m_LBP_RIU_VAR, pHistEntry);
	//}

	////texture descriptor LBP RIU
	//fread(&this->m_lbpP, sizeof(int), 1, fp);
	//fread(&this->m_lbpR, sizeof(int), 1, fp);
	//fread(&this->m_noUsedLbpPts, sizeof(int), 1, fp);
	//fread(&noBinsUsed, sizeof(int), 1, fp);

	//RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem0, RVLQLIST, this->m_LBP_RIU);
	//RVLQLIST_INIT(this->m_LBP_RIU);

	//for (int i = 0; i < noBinsUsed; i++)
	//{
	//	RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem0, RVLQLIST_HIST_ENTRY, pHistEntry);  //added by Karlo 
	//	fread(pHistEntry, sizeof(RVLQLIST_HIST_ENTRY), 1, fp);
	//	RVLQLIST_ADD_ENTRY(this->m_LBP_RIU, pHistEntry);
	//}

	////texture descriptor LBP
	//noBinsUsed = 0;
	//fread(&this->m_lbpP, sizeof(int), 1, fp);
	//fread(&this->m_lbpR, sizeof(int), 1, fp);
	//fread(&this->m_noUsedLbpPts, sizeof(int), 1, fp);
	//fread(&noBinsUsed, sizeof(int), 1, fp);

	//RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem0, RVLQLIST, this->m_LBP);
	//RVLQLIST_INIT(this->m_LBP);

	//for (int i = 0; i < noBinsUsed; i++)
	//{
	//	RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem0, RVLQLIST_HIST_ENTRY, pHistEntry);
	//	fread(pHistEntry, sizeof(RVLQLIST_HIST_ENTRY), 1, fp);
	//	RVLQLIST_ADD_ENTRY(this->m_LBP, pHistEntry);
	//}
}

void CRVL3DMeshObject::GetPointNeighbourhood(int *neighbourhood,
											 int x,
											 int y,
											 int p,
											 int r,
											 IplImage* grayImg)
{
	float xt = 0.0;
	float yt = 0.0;
	int k = (p / 4);
	int brX = 0;
	int brY = 0;
	uchar *ptr;
	for (int i = 0; i < p; i++)
	{
		xt = (float)(-1 * r * sin(2 * PI * i / p));
		yt = (float)(r * cos(2 * PI * i / p));
		
		if (xt == 0.0)
			xt = abs(xt);
		if (yt == 0.0)
			yt = abs(yt);
		xt += x;
		yt += y;
		brX = floor(xt+0.5);
		brY = floor(yt+0.5);

		/*if (xt > 0)
		{			
			xt -= 0.5;
			brX = ceil(xt);
			xt = x + brX + (xt - floor(xt));
		}
		else if(xt < 0)
		{			
			xt += 0.5;
			brX = ceil(abs(xt));
			xt = x - brX + abs(brX + xt);
		}

		if (yt > 0)
		{
			yt -= 0.5;
			brY = ceil(yt);
			yt = y + brY + (yt - floor(yt));
		}
		else if (yt < 0)
		{
			yt += 0.5;
			brY = ceil(abs(yt));
			yt = y - brY + abs(brY + yt);
		}*/

		if (i % k == 0)
		{
			ptr = (uchar*)(grayImg->imageData + brY * grayImg->widthStep);//(uchar*)(grayImg->imageData + (y+brY) * grayImg->widthStep);
			neighbourhood[i] = ptr[brX];//ptr[x+brX];
		}
		else
		{
			//xt = brX; //ZASTO OVO!?!?
			neighbourhood[i] = GetBiLinInterPixVal(xt, yt, grayImg);
		}

	}
}

int CRVL3DMeshObject::GetBiLinInterPixVal(float x,
										  float y,
										  IplImage* grayImg)
{
	int xf = floor(x);
	float dx = abs(x - float(xf));
	int yf = floor(y);
	float dy = abs(y - float(yf));
	uchar *ptr;
	ptr = (uchar*)(grayImg->imageData + (yf-1) * grayImg->widthStep);
	int I01 = ptr[xf-1];	//gornji lijevi
	ptr = (uchar*)(grayImg->imageData + (yf-1) * grayImg->widthStep);
	int I11 = ptr[xf+1];	//gornji desni
	ptr = (uchar*)(grayImg->imageData + (yf+1) * grayImg->widthStep);
	int I00 = ptr[xf-1];	//donji lijevi
	ptr = (uchar*)(grayImg->imageData + (yf+1) * grayImg->widthStep);
	int I10 = ptr[xf+1];	//donji desni

	//KRIVO?return (int)((float)I11 * (x - (xf-1)) * (y - (yf-1)) * 0.25 + (float)I21*((xf+1) - x) * (y - (yf-1)) * 0.25 + (float)I12*(x - (xf-1)) * ((yf+1) - y) * 0.25 + (float)I22*((xf+1) - x) * ((yf+1) - y) * 0.25);
	//KRIVO???OPET???return (int)(I00 * (1 - dx) * (1 - dy) + I10 * dx * (1 - dy) + I01 * (1 - dx) * dy + I11 * dx * dy);
	return (int)(I01 * (1 - dx) * (1 - dy) + I11 * dx * (1 - dy) + I00 * (1 - dx) * dy + I10 * dx * dy);
}

void CRVL3DMeshObject::CalculateLbp(int p,
									int r,
									IplImage* grayImg,
									bool soft)
{
	if (r > 3)	//Za sada je dopusteno samo R = 1,2,3
		return;
	//int hist_bins = (int)pow(2.0,p);
	int hist_bins = (int)(pow((float)p,2) / 2);
	int binW = (int)(pow(2.0,p) / hist_bins);
	float *lbpHistTriangle;
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, float, hist_bins, lbpHistTriangle);
	memset(lbpHistTriangle, 0, hist_bins * sizeof(float));

	float *lbpHistObject;
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, float, hist_bins, lbpHistObject);
	memset(lbpHistObject, 0, hist_bins * sizeof(float));

	CRVL2DRegion2 *pTriangle;
	RVL3DPOINT2 **ppPt, **pPtArrayEnd;
	RVL3DPOINT2 *pPt;
	int iPix, iPixD;
	int label;
	int x, y, w = grayImg->width, h = grayImg->height;
	int lbp = 0;
	int *neighbourhood;
	float binCentar = 0.0;	//Needed for soft histogram, centar of main bin
	int updateBin = 0;	//Needed for soft histogram, address of bin tu update(beside main bin)
	float binWeight = 0.0;	//Needed for soft histogram, weight of the main bin
	int bin = 0;
	uchar *ptr;
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, int, p, neighbourhood);
	memset(neighbourhood, 0, p * sizeof(int));

	RVLQLIST_PTR_ENTRY *pElement;
	pElement = (RVLQLIST_PTR_ENTRY*)this->m_FaceList->pFirst;

	this->m_noUsedLbpPts = 0; // Za sve LBP-ove bi trebalo biti isti ovaj broj(Zbog iste P i R konfiguracije)

	while(pElement)
	{
		pTriangle = (CRVL2DRegion2 *)(pElement->Ptr);

		if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		label = pTriangle->m_Label;

		pPtArrayEnd = pTriangle->m_pPoint3DArray + pTriangle->m_n3DPts;

		memset(lbpHistTriangle, 0, hist_bins * sizeof(float));

		RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST, pTriangle->m_LBP);
		RVLQLIST_INIT(pTriangle->m_LBP);
		//Stvaramo histogram na razini trokuta
		for(ppPt = pTriangle->m_pPoint3DArray; ppPt < pPtArrayEnd; ppPt++)
		{
			pPt = *ppPt;
			
			iPixD = pPt->iPix;
			iPix = pPt->iPixRGB;
			//ako je maska postavljena i nije dozvoljen idemo dalje
			if (pTriangle->m_uPixMask && ((pTriangle->m_uPixMask[iPixD] == 1) || ((pTriangle->m_uPixMask[iPixD] == 2) && (r >= 1)) || ((pTriangle->m_uPixMask[iPixD] == 3) && (r >= 2)) || ((pTriangle->m_uPixMask[iPixD] == 4) && (r >= 3))))
				continue;
			
			//Povecavamo broj koristenih piksela koji su koristeni s LBP algoritmom
			this->m_noUsedLbpPts++;

			//Klasicni LBP algoritam
			memset(neighbourhood, 0, p * sizeof(int));
			y = iPix / w;
			x = iPix - y * w;
			if (y != 0)
				x --;
			ptr = (uchar*)(grayImg->imageData + y * grayImg->widthStep);
			this->GetPointNeighbourhood(neighbourhood, x, y, p, r, grayImg);
			lbp = 0;
			for (int i = 0; i < p; i++)
			{
				if (neighbourhood[i] >= ptr[x])
					lbp += pow(2.0, i);
			}
			bin = (int)floor((float)(lbp/binW));
			if (!soft)
				lbpHistTriangle[bin]++;
			else
			{
				binCentar = bin * binW + binW / 2.0;
				binWeight = (float)(1.0 - abs(lbp - binCentar) / binW);
				lbpHistTriangle[bin] += binWeight;
				if (lbp > binCentar)
				{
					if ((bin + 1) < hist_bins)
						lbpHistTriangle[bin + 1] += 1 - binWeight;//(float)(1.0 - abs(lbp - (binCentar + binW) / binW));
					else
						lbpHistTriangle[bin] += 1 - binWeight;

				}
				else if (lbp < binCentar)
				{
					if ((bin - 1) >= 0)
						lbpHistTriangle[bin - 1] += 1 - binWeight;//(float)(1.0 - abs(lbp - (binCentar - binW) / binW));
					else
						lbpHistTriangle[bin] += 1 - binWeight;
				}
			}

		}
		//Postavljamo u listu histogram na razini trokuta i azuriramo na razini objekta
		for(int i = 0; i < hist_bins; i++)
		{
			if (lbpHistTriangle[i] != 0)
			{
				RVLQLIST_HIST_ENTRY *pHistEntry;
				RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST_HIST_ENTRY, pHistEntry);
				pHistEntry->adr = i;
				pHistEntry->value = lbpHistTriangle[i];
				RVLQLIST_ADD_ENTRY(pTriangle->m_LBP, pHistEntry);
				lbpHistObject[i] += lbpHistTriangle[i];
			}
		}
		pElement = (RVLQLIST_PTR_ENTRY*)pElement->pNext;
	}

	//Popunjavamo listu histograma i azuriramo roditeljski ako postoji
	if (((CRVL3DMeshObject*)this->parentMeshObject)->m_LBP)
	{
		RVLQLIST_HIST_ENTRY *pEntryT;
		RVLQLIST_HIST_ENTRY *pEntryP;
		RVLQLIST *parentLBP;
		parentLBP = (RVLQLIST*)((CRVL3DMeshObject*)this->parentMeshObject)->m_LBP;
		pEntryT = (RVLQLIST_HIST_ENTRY*)(parentLBP)->pFirst;
		pEntryP = pEntryT;
		
		for(int i = 0; i < hist_bins; i++)
		{
			if (lbpHistObject[i] != 0)
			{
				RVLQLIST_HIST_ENTRY *pHistEntry;
				RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST_HIST_ENTRY, pHistEntry);
				pHistEntry->adr = i;
				pHistEntry->value = lbpHistObject[i];
				RVLQLIST_ADD_ENTRY(this->m_LBP, pHistEntry);
				
				//Provjeravamo s parentom
				while (pEntryT->adr < i)
				{
					pEntryP = pEntryT;
					pEntryT = (RVLQLIST_HIST_ENTRY*)pEntryT->pNext;
				}
				if (pEntryT->adr = i)
					pEntryT->value += lbpHistObject[i];
				else if (pEntryT->adr > i)
				{
					RVLQLIST_HIST_ENTRY *pHistNewEntry;
					RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST_HIST_ENTRY, pHistNewEntry);
					pHistNewEntry->adr = i;
					pHistNewEntry->value = lbpHistObject[i];
					RVLQLIST_INSERT_ENTRY(parentLBP, pEntryP, pEntryT, pHistNewEntry);
				}

			}
		}				
	}
	else
	{
		for(int i = 0; i < hist_bins; i++)
		{
			if (lbpHistObject[i] != 0)
			{
				RVLQLIST_HIST_ENTRY *pHistEntry;
				RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST_HIST_ENTRY, pHistEntry);
				pHistEntry->adr = i;
				pHistEntry->value = lbpHistObject[i];
				RVLQLIST_ADD_ENTRY(this->m_LBP, pHistEntry);
			}
		}
	}
	this->m_lbpP = p;
	this->m_lbpR = r;
}

void CRVL3DMeshObject::CalculateLbpRiu(int p,
									   int r,
									   IplImage* grayImg)
{
	if (r > 3)	//Za sada je dopusteno samo R = 1,2,3
		return;
	int hist_bins = p + 2;
	ushort *lbpRIUHistTriangle;
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, ushort, hist_bins, lbpRIUHistTriangle);
	memset(lbpRIUHistTriangle, 0, hist_bins * sizeof(ushort));

	ushort *lbpRIUHistObject;
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, ushort, hist_bins, lbpRIUHistObject);
	memset(lbpRIUHistObject, 0, hist_bins * sizeof(ushort));

	CRVL2DRegion2 *pTriangle;
	RVL3DPOINT2 **ppPt, **pPtArrayEnd;
	RVL3DPOINT2 *pPt;
	int iPix, iPixD;
	int label;
	int x, y, w = grayImg->width, h = grayImg->height;
	int lbpRIU = 0;
	int pp = 0;
	int steps = 0;
	int *neighbourhood;
	uchar *ptr;
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, int, p, neighbourhood);
	memset(neighbourhood, 0, p * sizeof(int));

	RVLQLIST_PTR_ENTRY *pElement;
	pElement = (RVLQLIST_PTR_ENTRY*)this->m_FaceList->pFirst;

	this->m_noUsedLbpPts = 0; // Za sve LBP-ove bi trebalo biti isti ovaj broj

	while(pElement)
	{
		pTriangle = (CRVL2DRegion2 *)(pElement->Ptr);

		if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		label = pTriangle->m_Label;

		pPtArrayEnd = pTriangle->m_pPoint3DArray + pTriangle->m_n3DPts;

		memset(lbpRIUHistTriangle, 0, hist_bins * sizeof(ushort));

		RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST, pTriangle->m_LBP_RIU);
		RVLQLIST_INIT(pTriangle->m_LBP_RIU);
		//Stvaramo histogram na razini trokuta
		for(ppPt = pTriangle->m_pPoint3DArray; ppPt < pPtArrayEnd; ppPt++)
		{
			pPt = *ppPt;
			
			iPixD = pPt->iPix;
			iPix = pPt->iPixRGB;
			//ako je maska postavljena i nije dozvoljen idemo dalje
			if (pTriangle->m_uPixMask && ((pTriangle->m_uPixMask[iPixD] == 1) || ((pTriangle->m_uPixMask[iPixD] == 2) && (r >= 1)) || ((pTriangle->m_uPixMask[iPixD] == 3) && (r >= 2)) || ((pTriangle->m_uPixMask[iPixD] == 4) && (r >= 3))))
				continue;

			//Povecavamo broj koristenih piksela koji su koristeni s LBP algoritmom
			this->m_noUsedLbpPts++;

			//Rotational invariant with uniform regions LBP algoritam
			memset(neighbourhood, 0, p * sizeof(int));
			y = iPix / w;
			x = iPix - y * w;
			if (y != 0)
				x --;
			ptr = (uchar*)(grayImg->imageData + y * grayImg->widthStep);
			this->GetPointNeighbourhood(neighbourhood, x, y, p, r, grayImg);
			steps = 0;
			pp = 0;
			lbpRIU = 0;
			for (int i = 0; i < p; i++)
			{
				if (neighbourhood[i] >= ptr[x])
				{
					if ((pp == 0) && (i != 0))	//ako prosla tocka nije bila veca a ova je
						steps++;
					lbpRIU++;
					pp = 1;
				}
				else
				{
					if (pp == 1)	//ako je prosla tocka bila veca a ova nije
						steps++;
					pp = 0;
				}
			}
			if (steps > 2)	//ako je broj promjena veci od 2 labela je "p+1" a ako nije onda je labela jednaka broju tcaka vecih od sredisnje(ili jednakih)
				lbpRIU = p + 1;

			lbpRIUHistTriangle[lbpRIU]++;

		}
		//Postavljamo u listu histogram na razini trokuta i azuriramo na razini objekta
		for(int i = 0; i < hist_bins; i++)
		{
			if (lbpRIUHistTriangle[i] != 0)
			{
				RVLQLIST_HIST_ENTRY *pHistEntry;
				RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST_HIST_ENTRY, pHistEntry);
				pHistEntry->adr = i;
				pHistEntry->value = lbpRIUHistTriangle[i];
				RVLQLIST_ADD_ENTRY(pTriangle->m_LBP_RIU, pHistEntry);
				lbpRIUHistObject[i] += lbpRIUHistTriangle[i];
			}
		}
		pElement = (RVLQLIST_PTR_ENTRY*)pElement->pNext;
	}

	//Popunjavamo listu histograma i azuriramo roditeljski ako postoji
	if (((CRVL3DMeshObject*)this->parentMeshObject)->m_LBP_RIU)
	{
		RVLQLIST_HIST_ENTRY *pEntryT;
		RVLQLIST_HIST_ENTRY *pEntryP;
		RVLQLIST *parentLBPRIU;
		parentLBPRIU = (RVLQLIST*)((CRVL3DMeshObject*)this->parentMeshObject)->m_LBP_RIU;
		pEntryT = (RVLQLIST_HIST_ENTRY*)(parentLBPRIU)->pFirst;
		pEntryP = pEntryT;
		
		for(int i = 0; i < hist_bins; i++)
		{
			if (lbpRIUHistObject[i] != 0)
			{
				RVLQLIST_HIST_ENTRY *pHistEntry;
				RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST_HIST_ENTRY, pHistEntry);
				pHistEntry->adr = i;
				pHistEntry->value = lbpRIUHistObject[i];
				RVLQLIST_ADD_ENTRY(this->m_LBP_RIU, pHistEntry);
				
				//Provjeravamo s parentom
				while (pEntryT->adr < i)
				{
					pEntryP = pEntryT;
					pEntryT = (RVLQLIST_HIST_ENTRY*)pEntryT->pNext;
				}
				if (pEntryT->adr = i)
					pEntryT->value += lbpRIUHistObject[i];
				else if (pEntryT->adr > i)
				{
					RVLQLIST_HIST_ENTRY *pHistNewEntry;
					RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST_HIST_ENTRY, pHistNewEntry);
					pHistNewEntry->adr = i;
					pHistNewEntry->value = lbpRIUHistObject[i];
					RVLQLIST_INSERT_ENTRY(parentLBPRIU, pEntryP, pEntryT, pHistNewEntry);
				}

			}
		}				
	}
	else
	{
		for(int i = 0; i < hist_bins; i++)
		{
			if (lbpRIUHistObject[i] != 0)
			{
				RVLQLIST_HIST_ENTRY *pHistEntry;
				RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST_HIST_ENTRY, pHistEntry);
				pHistEntry->adr = i;
				pHistEntry->value = lbpRIUHistObject[i];
				RVLQLIST_ADD_ENTRY(this->m_LBP_RIU, pHistEntry);
			}
		}
	}
	this->m_lbpP = p;
	this->m_lbpR = r;
}

void CRVL3DMeshObject::CalculateLbpRiuVar(int p,
										  int r,
										  float maxVar,
										  IplImage* grayImg,
										  bool soft)
{
	if (r > 3)	//Za sada je dopusteno samo R = 1,2,3
		return;
	int hist_bins_lbp = p + 2;
	int hist_bins_var = 16 + 1;
	maxVar = maxVar;
	float varBinW = maxVar / (hist_bins_var - 1);
	float *lbpRiuVarHistTriangle;
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, float, hist_bins_lbp * hist_bins_var, lbpRiuVarHistTriangle);
	memset(lbpRiuVarHistTriangle, 0, hist_bins_lbp * hist_bins_var * sizeof(float));

	float *lbpRiuVarHistObject;
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, float, hist_bins_lbp * hist_bins_var, lbpRiuVarHistObject);
	memset(lbpRiuVarHistObject, 0, hist_bins_lbp * hist_bins_var * sizeof(float));

	CRVL2DRegion2 *pTriangle;
	RVL3DPOINT2 **ppPt, **pPtArrayEnd;
	RVL3DPOINT2 *pPt;
	int iPix, iPixD;
	int label;
	int x, y, w = grayImg->width, h = grayImg->height;
	int lbpRIU = 0;
	float var = 0.0;
	int varBin = 0;
	float avg = 0.0;
	int pp = 0;
	int steps = 0;
	int *neighbourhood;
	float varBinCentar = 0.0;	//Needed for soft histogram, centar of main bin
	int updateBin = 0;	//Needed for soft histogram, address of bin tu update(beside main bin)
	float varBinWeight = 0.0;	//Needed for soft histogram, weight for main bin
	uchar *ptr;
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, int, p, neighbourhood);
	memset(neighbourhood, 0, p * sizeof(int));

	RVLQLIST_PTR_ENTRY *pElement;
	pElement = (RVLQLIST_PTR_ENTRY*)this->m_FaceList->pFirst;

	this->m_noUsedLbpPts = 0; // Za sve LBP-ove bi trebalo biti isti ovaj broj

	while(pElement)
	{
		pTriangle = (CRVL2DRegion2 *)(pElement->Ptr);

		if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		label = pTriangle->m_Label;

		pPtArrayEnd = pTriangle->m_pPoint3DArray + pTriangle->m_n3DPts;

		memset(lbpRiuVarHistTriangle, 0, hist_bins_lbp * hist_bins_var * sizeof(float));

		RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST, pTriangle->m_LBP_RIU_VAR);
		RVLQLIST_INIT(pTriangle->m_LBP_RIU_VAR);
		//Stvaramo histogram na razini trokuta
		for(ppPt = pTriangle->m_pPoint3DArray; ppPt < pPtArrayEnd; ppPt++)
		{
			pPt = *ppPt;
			
			iPixD = pPt->iPix;
			iPix = pPt->iPixRGB;
			//ako je maska postavljena i nije dozvoljen idemo dalje
			if (pTriangle->m_uPixMask && ((pTriangle->m_uPixMask[iPixD] == 1) || ((pTriangle->m_uPixMask[iPixD] == 2) && (r >= 1)) || ((pTriangle->m_uPixMask[iPixD] == 3) && (r >= 2)) || ((pTriangle->m_uPixMask[iPixD] == 4) && (r >= 3))))
				continue;

			//Povecavamo broj koristenih piksela koji su koristeni s LBP algoritmom
			this->m_noUsedLbpPts++;

			//Rotational invariant with uniform regions LBP algoritam
			memset(neighbourhood, 0, p * sizeof(int));
			y = iPix / w;
			x = iPix - y * w;
			if (y != 0)
				x --;
			ptr = (uchar*)(grayImg->imageData + y * grayImg->widthStep);
			this->GetPointNeighbourhood(neighbourhood, x, y, p, r, grayImg);
			steps = 0;
			pp = 0;
			lbpRIU = 0;
			var = 0.0;
			avg = 0.0;
			varBin = 0;
			//nalazimo srednju vrijednost okoline
			for (int i = 0; i < p; i++)
			{
				avg += neighbourhood[i];
			}
			avg /= (float)p;

			for (int i = 0; i < p; i++)
			{
				if (neighbourhood[i] >= ptr[x])
				{
					if ((pp == 0) && (i != 0))	//ako prosla tocka nije bila veca a ova je
						steps++;
					lbpRIU++;
					pp = 1;
				}
				else
				{
					if (pp == 1)	//ako je prosla tocka bila veca a ova nije
						steps++;
					pp = 0;
				}
				var += pow((float)(neighbourhood[i] - avg), 2);
			}
			if (steps > 2)	//ako je broj promjena veci od 2 labela je "p+1" a ako nije onda je labela jednaka broju tcaka vecih od sredisnje(ili jednakih)
				lbpRIU = p + 1;
			var /= (float)p;
			varBin = (int)floor(var/varBinW);
			if (var >= maxVar)
				varBin = hist_bins_var - 1;

			if (!soft)
				lbpRiuVarHistTriangle[varBin * hist_bins_lbp + lbpRIU]++;
			else
			{
				varBinCentar = varBin * varBinW + varBinW / 2.0;
				if (var > maxVar)
					var = maxVar;
				varBinWeight = (float)(1.0 - abs(var - varBinCentar) / varBinW);
				lbpRiuVarHistTriangle[varBin * hist_bins_lbp + lbpRIU] += varBinWeight;
				if (var > varBinCentar)
				{
					if((varBin + 1) < hist_bins_var)
						lbpRiuVarHistTriangle[(varBin + 1) * hist_bins_lbp + lbpRIU] += 1.0 - varBinWeight;
					else
						lbpRiuVarHistTriangle[varBin * hist_bins_lbp + lbpRIU] += 1.0 - varBinWeight;
				}
				else if (var < varBinCentar)
				{
					if ((varBin - 1) >= 0)
						lbpRiuVarHistTriangle[(varBin - 1) * hist_bins_lbp + lbpRIU] += 1.0 - varBinWeight;
					else
						lbpRiuVarHistTriangle[varBin * hist_bins_lbp + lbpRIU] += 1 - varBinWeight;
				}
			}

		}
		//Postavljamo u listu histogram na razini trokuta i azuriramo na razini objekta
		for(int i = 0; i < hist_bins_lbp * hist_bins_var; i++)
		{
			if (lbpRiuVarHistTriangle[i] != 0)
			{
				RVLQLIST_HIST_ENTRY *pHistEntry;
				RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST_HIST_ENTRY, pHistEntry);
				pHistEntry->adr = i;
				pHistEntry->value = lbpRiuVarHistTriangle[i];
				RVLQLIST_ADD_ENTRY(pTriangle->m_LBP_RIU_VAR, pHistEntry);
				lbpRiuVarHistObject[i] += lbpRiuVarHistTriangle[i];
			}
		}
		pElement = (RVLQLIST_PTR_ENTRY*)pElement->pNext;
	}

	//Popunjavamo listu histograma i azuriramo roditeljski ako postoji
	if (((CRVL3DMeshObject*)this->parentMeshObject)->m_LBP_RIU)
	{
		RVLQLIST_HIST_ENTRY *pEntryT;
		RVLQLIST_HIST_ENTRY *pEntryP;
		RVLQLIST *parentLBPRiuVar;
		parentLBPRiuVar = (RVLQLIST*)((CRVL3DMeshObject*)this->parentMeshObject)->m_LBP_RIU_VAR;
		pEntryT = (RVLQLIST_HIST_ENTRY*)(parentLBPRiuVar)->pFirst;
		pEntryP = pEntryT;
		
		for(int i = 0; i < hist_bins_lbp * hist_bins_var; i++)
		{
			if (lbpRiuVarHistObject[i] != 0)
			{
				RVLQLIST_HIST_ENTRY *pHistEntry;
				RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST_HIST_ENTRY, pHistEntry);
				pHistEntry->adr = i;
				pHistEntry->value = lbpRiuVarHistObject[i];
				RVLQLIST_ADD_ENTRY(this->m_LBP_RIU_VAR, pHistEntry);
				
				//Provjeravamo s parentom
				while (pEntryT->adr < i)
				{
					pEntryP = pEntryT;
					pEntryT = (RVLQLIST_HIST_ENTRY*)pEntryT->pNext;
				}
				if (pEntryT->adr = i)
					pEntryT->value += lbpRiuVarHistObject[i];
				else if (pEntryT->adr > i)
				{
					RVLQLIST_HIST_ENTRY *pHistNewEntry;
					RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST_HIST_ENTRY, pHistNewEntry);
					pHistNewEntry->adr = i;
					pHistNewEntry->value = lbpRiuVarHistObject[i];
					RVLQLIST_INSERT_ENTRY(parentLBPRiuVar, pEntryP, pEntryT, pHistNewEntry);
				}

			}
		}				
	}
	else
	{
		for(int i = 0; i < hist_bins_lbp * hist_bins_var; i++)
		{
			if (lbpRiuVarHistObject[i] != 0)
			{
				RVLQLIST_HIST_ENTRY *pHistEntry;
				RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST_HIST_ENTRY, pHistEntry);
				pHistEntry->adr = i;
				pHistEntry->value = lbpRiuVarHistObject[i];
				RVLQLIST_ADD_ENTRY(this->m_LBP_RIU_VAR, pHistEntry);
			}
		}
	}
	this->m_lbpP = p;
	this->m_lbpR = r;
}
float CRVL3DMeshObject::RVLIntersectHistograms(RVLQLIST *histS,
										RVLQLIST *histM,
										int noUsedPtsS,
										int noUsedPtsM,
										int maxNoBins,
										CRVLMem *pMem,
										int usedBins,
										float *helperArray)
{
	//ako nepostoje odgovarajuci histogrami ili za nihovo stvaranje nije koristen ni jedan piksel u boji prekini
	if (!histS || !histM || noUsedPtsS == 0 || noUsedPtsM == 0)
		return 0.0;
	
	float intersectValue = 0.0;

	if (usedBins == -1)
	{
		usedBins = maxNoBins;
		//float *objValues;
		//RVLMEM_ALLOC_STRUCT_ARRAY(pMem, float, maxNoBins, objValues);
		memset(helperArray, 0, maxNoBins * sizeof(float));
		RVLQLIST_HIST_ENTRY *pHistEntry;
		pHistEntry = (RVLQLIST_HIST_ENTRY*)histS->pFirst;
		while(pHistEntry)
		{
			helperArray[pHistEntry->adr] = (float)pHistEntry->value / (float)noUsedPtsS;
			pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
		}
		pHistEntry = (RVLQLIST_HIST_ENTRY*)histM->pFirst;
		float modelValue = 0.0;
		while(pHistEntry)
		{
			modelValue = (float)pHistEntry->value / (float)noUsedPtsM;
			if (modelValue < helperArray[pHistEntry->adr])
				intersectValue += modelValue;
			else
				intersectValue += helperArray[pHistEntry->adr];
			pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
		}
	}
	else
	{
		float *objValues;
		RVLMEM_ALLOC_STRUCT_ARRAY(pMem, float, usedBins, objValues);
		memset(objValues, 0, usedBins * sizeof(float));
		int *objBinIdx;
		RVLMEM_ALLOC_STRUCT_ARRAY(pMem, int, usedBins, objBinIdx);
		memset(objBinIdx, 0, usedBins * sizeof(int));

		RVLQLIST_HIST_ENTRY *pHistEntry;
		pHistEntry = (RVLQLIST_HIST_ENTRY*)histS->pFirst;
		int foundObjBins = 0;
		for (int i = 0; i < usedBins; i++)
		{
			objValues[i] = (float)pHistEntry->value / (float)noUsedPtsS;
			objBinIdx[i] = pHistEntry->adr;
			foundObjBins++;
			pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
			if (!pHistEntry)
				break;
		}
		int br = 0;
		pHistEntry = (RVLQLIST_HIST_ENTRY*)histM->pFirst;
		float modelValue = 0.0;
		int i;
		while(pHistEntry)
		{
			for (i = 0; i < foundObjBins; i++)
			{
				if (pHistEntry->adr == objBinIdx[i])
				{
					modelValue = (float)pHistEntry->value / (float)noUsedPtsM;
					if (modelValue < objValues[i])
						intersectValue += modelValue;
					else
						intersectValue += objValues[i];
					br++;
					break;
				}
			}
			if (br == foundObjBins)
				break;
			pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
		}
	}
	return intersectValue;
}

void CRVL3DMeshObject::RVLCalculateColorHist(IplImage* colorImg, float* histBase, bool soft)
{
	if ((colorImg->channelSeq[0] == 'B') || (colorImg->channelSeq[0] == 'R'))
		this->RVLCalculateRGBHist(colorImg, histBase, soft);
	else if (colorImg->channelSeq[0] == 'X')
		this->RVLCalculateXYZHist(colorImg, histBase, soft);
	else if (colorImg->channelSeq[0] == 'H')
		this->RVLCalculateHSVHist(colorImg, histBase, soft);
	else if (colorImg->channelSeq[0] == 'L')
		this->RVLCalculateLabHist(colorImg, histBase, soft);
}

void CRVL3DMeshObject::RVLCalculateHSVHist(IplImage* hsvImg, float* histBase, bool soft)
{
	int noBins = (int)histBase[0] * histBase[1];
	float ratioS = 256 / histBase[0]; //how many levels are per bin for Saturation
	float ratioH = 181 / histBase[1]; //levels per bin for Hue
	CRVL2DRegion2 *pTriangle;
	RVL3DPOINT2 **ppPt, **pPtArrayEnd;
	RVL3DPOINT2 *pPt;
	int iPix;
	int label;
	int h = 0, s = 0, rgb_x = 0, rgb_y = 0;
	int width = hsvImg->width;
	int height = hsvImg->height;
	int adr;

	int updateBins[4][2];	//Needed for soft histograms, coordinates for updated bins
	memset(updateBins, 0, 4 * 2 * sizeof(int));
	int bins2Update = 0;	//Needed for soft histograms, number of bins needed to update
	float binCentar[2];	//Needed for soft histograms, closest bin centar
	int bin[2];	//Needed for soft histograms, central bin coordinates
	int valuePos[2];	//current value above, below or on bin centar, per dimension(R,G,B)
	float binWeightCoeff[4][2];	//Needed for soft histograms, calculated coefficients for faster analysis
	float weightTypes[2][2];	//Needed for soft histograms, there can be only two weights, offset by 0 or +/- 1
	memset(updateBins, 0, 4 * 2 * sizeof(float));

	void** histHSV;
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, void*, noBins, histHSV);
	RVLQLIST *histHSVList;
	RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST, histHSVList);

	void** objHistHSV;
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, void*, noBins, objHistHSV);
	for(int i = 0; i < noBins; i++)
	{
		if (objHistHSV[i])
			objHistHSV[i] = NULL;
	}
	RVLQLIST *objHistHSVList;
	RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST, objHistHSVList);
	RVLQLIST_INIT(objHistHSVList);

	uchar *ptr;
	RVLQLIST_PTR_ENTRY *pElement;
	pElement = (RVLQLIST_PTR_ENTRY*)this->m_FaceList->pFirst;

	while(pElement)
	{
		pTriangle = (CRVL2DRegion2 *)(pElement->Ptr);

		if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		label = pTriangle->m_Label;

		pPtArrayEnd = pTriangle->m_pPoint3DArray + pTriangle->m_n3DPts;

		for(int i = 0; i < noBins; i++)
		{
			if (histHSV[i])
				histHSV[i] = NULL;
		}
		RVLQLIST_INIT(histHSVList);	//resetiramo listu 

		RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST, pTriangle->m_histRGB);
		RVLQLIST_INIT(pTriangle->m_histRGB);
		pTriangle->m_histRGB_base[0] = histBase[0];
		pTriangle->m_histRGB_base[1] = histBase[1];
		pTriangle->m_histRGB_base[2] = histBase[2];
		pTriangle->m_ColorSystem = RVL_MESH_COLOR_HSV_2D;

		for(ppPt = pTriangle->m_pPoint3DArray; ppPt < pPtArrayEnd; ppPt++)
		{
			pPt = *ppPt;

			iPix = pPt->iPix;
			//ako je maska postavljena i nije dozvoljen idemo dalje
			if (pTriangle->m_uPixMask && (pTriangle->m_uPixMask[iPix] == 1))
				continue;
			//Povecavamo broj koristenih piksela s bojom
			this->m_noUsedColorPts++;
			//Provjeriti
			rgb_y = pPt->iPixRGB / width;
			rgb_x = pPt->iPixRGB - rgb_y * width;
			if (rgb_y != 0)
				rgb_x --;
			ptr = (uchar*)(hsvImg->imageData + rgb_y * hsvImg->widthStep);
			h = ptr[rgb_x * 3];
			s = ptr[rgb_x * 3 + 1];

			if(!soft)
			{
				adr = (int)(floor(h / ratioH) * histBase[1] + floor(s / ratioS)); // i = h/(256/16) * 16 + s/(256/16)
				if (histHSV[adr])
				{
					//azuriramo lokalni trikutic
					((RVLQLIST_HIST_ENTRY*)histHSV[adr])->value++;
					//azuriramo objekt
					((RVLQLIST_HIST_ENTRY*)objHistHSV[adr])->value++;
				}
				else
				{
					//stvaramo novi element koji ce biti i u arrayu i u povezanom popisu
					RVLQLIST_HIST_ENTRY *pHistEntryT;
					RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST_HIST_ENTRY, pHistEntryT);
					pHistEntryT->adr = adr;
					pHistEntryT->value = 1;
					histHSV[adr] = pHistEntryT;
					RVLQLIST_ADD_ENTRY(histHSVList, pHistEntryT);
					
					if (!objHistHSV[adr])
					{
						RVLQLIST_HIST_ENTRY *pHistEntryO;
						RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST_HIST_ENTRY, pHistEntryO);
						pHistEntryO->adr = adr;
						pHistEntryO->value = 1;
						objHistHSV[adr] = pHistEntryO;
						RVLQLIST_ADD_ENTRY(objHistHSVList, pHistEntryO);
					}
					else
						((RVLQLIST_HIST_ENTRY*)objHistHSV[adr])->value++;	//azuriramo objekt
				}
			}
			else
			{
				//closest bin centar
				bin[0] = (int)floor(h / ratioH);
				bin[1] = (int)floor(s / ratioS);
				binCentar[0] = (float)(bin[0] * ratioH + ratioH / 2);
				binCentar[1] = (int)(bin[1] * ratioS + ratioS / 2);
				
				//is current value above bin centar per dimension
				if (h > binCentar[0])
					valuePos[0] = 1;
				else if (h == binCentar[0])
					valuePos[0] = 0;
				else if (h < binCentar[0])
					valuePos[0] = -1;

				if (s > binCentar[1])
					valuePos[1] = 1;
				else if (s == binCentar[1])
					valuePos[1] = 0;
				else if (s < binCentar[1])
					valuePos[1] = -1;

				//Finding which bin to update
				updateBins[0][0] = bin[0];
				updateBins[0][1] = bin[1];
				//Calculating weight types
				//'0' offset
				weightTypes[0][0] = (float)(1.0 - abs((float)(h - (updateBins[0][0] * ratioH + ratioH / 2.0))) / ratioH);
				weightTypes[0][1] = (float)(1.0 - abs((float)(s - (updateBins[0][1] * ratioS + ratioS / 2.0))) / ratioS);
				//'+/- 1' offset
				weightTypes[1][0] = (float)(1.0 - abs((float)(h - ((bin[0] + valuePos[0]) * ratioH + ratioH / 2.0))) / ratioH);
				weightTypes[1][1] = (float)(1.0 - abs((float)(s - ((bin[1] + valuePos[1]) * ratioS + ratioS / 2.0))) / ratioS);
				
				binWeightCoeff[0][0] = weightTypes[0][0];
				binWeightCoeff[0][1] = weightTypes[0][1];
				bins2Update = 1;

				if (!((valuePos[0] == 0) && (valuePos[1] == 0)))	//current color is in bin centar
				{
					//first coordinate
					if (valuePos[0] != 0)
					{
						updateBins[bins2Update][0] = bin[0] + valuePos[0];
						if ((updateBins[bins2Update][0] >= 0) && (updateBins[bins2Update][0] < histBase[0]))
						{
							updateBins[bins2Update][1] = bin[1];
							
							binWeightCoeff[bins2Update][0] = weightTypes[abs(bin[0] - updateBins[bins2Update][0])][0];
							binWeightCoeff[bins2Update][1] = weightTypes[abs(bin[1] - updateBins[bins2Update][1])][1];
							bins2Update++;

							//first & second coordinate
							updateBins[bins2Update][1] = bin[1] + valuePos[1];
							if ((updateBins[bins2Update][1] >= 0) && (updateBins[bins2Update][1] < histBase[1]))
							{
								updateBins[bins2Update][0] = bin[0] + valuePos[0];
								binWeightCoeff[bins2Update][0] = weightTypes[abs(bin[0] - updateBins[bins2Update][0])][0];
								binWeightCoeff[bins2Update][1] = weightTypes[abs(bin[1] - updateBins[bins2Update][1])][1];
								bins2Update++;
							}
						}
					}

					//second coordinate
					if (valuePos[1] != 0)
					{
						updateBins[bins2Update][1] = bin[1] + valuePos[1];
						if ((updateBins[bins2Update][1] >= 0)  && (updateBins[bins2Update][1] < histBase[1]))
						{
							updateBins[bins2Update][0] = bin[0];
							binWeightCoeff[bins2Update][0] = weightTypes[abs(bin[0] - updateBins[bins2Update][0])][0];
							binWeightCoeff[bins2Update][1] = weightTypes[abs(bin[1] - updateBins[bins2Update][1])][1];
							bins2Update++;
						}
					}
				}

				//Updating all taged bins
				for (int i = 0; i < bins2Update; i++)
				{
					adr = (int)(updateBins[i][0] * histBase[1] + updateBins[i][1]);

					if (histHSV[adr])
					{
						//azuriramo lokalni trikutic
						((RVLQLIST_HIST_ENTRY*)histHSV[adr])->value += binWeightCoeff[i][0] * binWeightCoeff[i][1];
						//azuriramo objekt
						((RVLQLIST_HIST_ENTRY*)objHistHSV[adr])->value += binWeightCoeff[i][0] * binWeightCoeff[i][1];
					}
					else
					{
						//stvaramo novi element koji ce biti i u arrayu i u povezanom popisu
						RVLQLIST_HIST_ENTRY *pHistEntryT;// = new RVLQLIST_HIST_ENTRY_SHORT[1];
						RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST_HIST_ENTRY, pHistEntryT);
						pHistEntryT->adr = adr;
						pHistEntryT->value = binWeightCoeff[i][0] * binWeightCoeff[i][1];
						histHSV[adr] = pHistEntryT;
						RVLQLIST_ADD_ENTRY(histHSVList, pHistEntryT);
						
						if (!objHistHSV[adr])
						{
							RVLQLIST_HIST_ENTRY *pHistEntryO;// = new RVLQLIST_HIST_ENTRY_SHORT[1];
							RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST_HIST_ENTRY, pHistEntryO);
							pHistEntryO->adr = adr;
							pHistEntryO->value = binWeightCoeff[i][0] * binWeightCoeff[i][1];
							objHistHSV[adr] = pHistEntryO;
							RVLQLIST_ADD_ENTRY(objHistHSVList, pHistEntryO);
						}
						else
							((RVLQLIST_HIST_ENTRY*)objHistHSV[adr])->value += binWeightCoeff[i][0] * binWeightCoeff[i][1];	//azuriramo objekt
					}

				}
			}		
		}
		//Stvaramo i sortiramo HSV histogram za trokut
		pTriangle->m_histRGB = InsertSortQLISTWithCopy<RVLQLIST_HIST_ENTRY>(histHSVList, RVLQLIST_DESCENDING, this->m_pClass->m_pMem0, TRUE);
		pElement = (RVLQLIST_PTR_ENTRY*)pElement->pNext;
	}
	//Stvaramo i sortiramo HSV histogram za cijeli objekt
	this->m_histRGB = InsertSortQLISTWithCopy<RVLQLIST_HIST_ENTRY>(objHistHSVList, RVLQLIST_DESCENDING, this->m_pClass->m_pMem0, TRUE);
	this->m_histRGB_base[0] = histBase[0];
	this->m_histRGB_base[1] = histBase[1];
	this->m_histRGB_base[2] = histBase[2];
	this->m_ColorSystem = RVL_MESH_COLOR_HSV_2D;

	//Azuriramo histogram od parenta ako postoji(NETESTIRANO!!!! STO AKO IMAJU RAZLICITE BAZE?!?!?)
	if (((CRVL3DMeshObject*)this->parentMeshObject)->m_histRGB)
	{
		RVLQLIST *pParentHistList = ((CRVL3DMeshObject*)this->parentMeshObject)->m_histRGB;
		if (pParentHistList->pFirst)
		{
			RVLQLIST_HIST_ENTRY *pParentHistEntry = (RVLQLIST_HIST_ENTRY *)pParentHistList->pFirst;
			RVLQLIST_HIST_ENTRY *pObjHistEntry = (RVLQLIST_HIST_ENTRY *)this->m_histRGB->pFirst;
			bool insert = FALSE;
			while(pObjHistEntry)
			{
				while (pParentHistEntry)
				{
					if (pObjHistEntry->adr == pParentHistEntry->adr)
						pParentHistEntry->value += pObjHistEntry->value;
					pParentHistEntry = (RVLQLIST_HIST_ENTRY *)pParentHistEntry->pNext;
					if (!pParentHistEntry)
					{
						RVLQLIST_ADD_ENTRY(pParentHistList, pObjHistEntry);
						insert = TRUE;
						break;
					}
				}
				pObjHistEntry = (RVLQLIST_HIST_ENTRY *)pObjHistEntry->pNext;
			}
			if (insert)
				((CRVL3DMeshObject*)this->parentMeshObject)->m_histRGB = InsertSortQLISTWithCopy<RVLQLIST_HIST_ENTRY>(pParentHistList, RVLQLIST_DESCENDING, this->m_pClass->m_pMem0, TRUE);

		}
		else
		{
			//Kopiraj podatke od objekta
			RVLQLIST_HIST_ENTRY *pObjHistEntry = (RVLQLIST_HIST_ENTRY *)this->m_histRGB->pFirst;
			while(pObjHistEntry)
			{
				RVLQLIST_HIST_ENTRY *pParentHistNewEntry;// = new RVLQLIST_HIST_ENTRY_SHORT[1];
				RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem0, RVLQLIST_HIST_ENTRY, pParentHistNewEntry);
				pParentHistNewEntry->adr = pObjHistEntry->adr;
				pParentHistNewEntry->value = pObjHistEntry->value;
				RVLQLIST_ADD_ENTRY(pParentHistList, pParentHistNewEntry);

				pObjHistEntry = (RVLQLIST_HIST_ENTRY *)pObjHistEntry->pNext;
			}
		}
	}
}

void CRVL3DMeshObject::RVLCalculateLabHist(IplImage* labImg, float* histBase, bool soft)
{
	int noBins = (int)histBase[0] * histBase[1];
	float ratioA = 256 / histBase[0]; //how many levels are per bin
	float ratioB = 256 / histBase[1]; //how many levels are per bin
	CRVL2DRegion2 *pTriangle;
	RVL3DPOINT2 **ppPt, **pPtArrayEnd;
	RVL3DPOINT2 *pPt;
	int iPix;
	int label;
	int a = 0, b = 0, rgb_x = 0, rgb_y = 0;
	int width = labImg->width;
	int height = labImg->height;
	int adr;

	int updateBins[4][2];	//Needed for soft histograms, coordinates for updated bins
	memset(updateBins, 0, 4 * 2 * sizeof(int));
	int bins2Update = 0;	//Needed for soft histograms, number of bins needed to update
	int binCentar[2];	//Needed for soft histograms, closest bin centar
	int bin[2];	//Needed for soft histograms, central bin coordinates
	int valuePos[2];	//current value above, below or on bin centar, per dimension(a,b)
	float binWeightCoeff[4][2];	//Needed for soft histograms, calculated coefficients for faster analysis
	float weightTypes[2][2];	//Needed for soft histograms, there can be only two weights, offset by 0 or +/- 1
	memset(updateBins, 0, 4 * 2 * sizeof(float));

	void** histLab;
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, void*, noBins, histLab);
	RVLQLIST *histLabList;
	RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST, histLabList);

	void** objHistLab;
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, void*, noBins, objHistLab);
	for(int i = 0; i < noBins; i++)
	{
		if (objHistLab[i])
			objHistLab[i] = NULL;
	}
	RVLQLIST *objHistLabList;
	RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST, objHistLabList);
	RVLQLIST_INIT(objHistLabList);

	uchar *ptr;
	RVLQLIST_PTR_ENTRY *pElement;
	pElement = (RVLQLIST_PTR_ENTRY*)this->m_FaceList->pFirst;

	while(pElement)
	{
		pTriangle = (CRVL2DRegion2 *)(pElement->Ptr);

		if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		label = pTriangle->m_Label;

		pPtArrayEnd = pTriangle->m_pPoint3DArray + pTriangle->m_n3DPts;

		for(int i = 0; i < noBins; i++)
		{
			if (histLab[i])
				histLab[i] = NULL;
		}
		RVLQLIST_INIT(histLabList);	//resetiramo listu 

		RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST, pTriangle->m_histRGB);
		RVLQLIST_INIT(pTriangle->m_histRGB);
		pTriangle->m_histRGB_base[0] = histBase[0];
		pTriangle->m_histRGB_base[1] = histBase[1];
		pTriangle->m_histRGB_base[2] = histBase[2];
		pTriangle->m_ColorSystem = RVL_MESH_COLOR_CIELAB_2D;

		for(ppPt = pTriangle->m_pPoint3DArray; ppPt < pPtArrayEnd; ppPt++)
		{
			pPt = *ppPt;

			iPix = pPt->iPix;
			//ako je maska postavljena i nije dozvoljen idemo dalje
			if (pTriangle->m_uPixMask && (pTriangle->m_uPixMask[iPix] == 1))
				continue;
			//Povecavamo broj koristenih piksela s bojom
			this->m_noUsedColorPts++;
			//Provjeriti
			rgb_y = pPt->iPixRGB / width;
			rgb_x = pPt->iPixRGB - rgb_y * width;
			if (rgb_y != 0)
				rgb_x --;
			ptr = (uchar*)(labImg->imageData + rgb_y * labImg->widthStep);
			a = ptr[rgb_x * 3 + 1];
			b = ptr[rgb_x * 3 + 2];

			if(!soft)
			{
				adr = (int)(floor(a / ratioA) * histBase[1] + floor(b / ratioB)); // i = a/(256/16) * 16 + a/(256/16)
				if (histLab[adr])
				{
					//azuriramo lokalni trikutic
					((RVLQLIST_HIST_ENTRY*)histLab[adr])->value++;
					//azuriramo objekt
					((RVLQLIST_HIST_ENTRY*)objHistLab[adr])->value++;
				}
				else
				{
					//stvaramo novi element koji ce biti i u arrayu i u povezanom popisu
					RVLQLIST_HIST_ENTRY *pHistEntryT;
					RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST_HIST_ENTRY, pHistEntryT);
					pHistEntryT->adr = adr;
					pHistEntryT->value = 1;
					histLab[adr] = pHistEntryT;
					RVLQLIST_ADD_ENTRY(histLabList, pHistEntryT);
					
					if (!objHistLab[adr])
					{
						RVLQLIST_HIST_ENTRY *pHistEntryO;
						RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST_HIST_ENTRY, pHistEntryO);
						pHistEntryO->adr = adr;
						pHistEntryO->value = 1;
						objHistLab[adr] = pHistEntryO;
						RVLQLIST_ADD_ENTRY(objHistLabList, pHistEntryO);
					}
					else
						((RVLQLIST_HIST_ENTRY*)objHistLab[adr])->value++;	//azuriramo objekt
				}
			}
			else
			{
				//closest bin centar
				bin[0] = (int)floor(a / ratioA);
				bin[1] = (int)floor(b / ratioB);
				binCentar[0] = (int)(bin[0] * ratioA + ratioA / 2);
				binCentar[1] = (int)(bin[1] * ratioB + ratioB / 2);
				
				//is current value above bin centar per dimension
				if (a > binCentar[0])
					valuePos[0] = 1;
				else if (a == binCentar[0])
					valuePos[0] = 0;
				else if (a < binCentar[0])
					valuePos[0] = -1;

				if (b > binCentar[1])
					valuePos[1] = 1;
				else if (b == binCentar[1])
					valuePos[1] = 0;
				else if (b < binCentar[1])
					valuePos[1] = -1;

				//Finding which bin to update
				updateBins[0][0] = bin[0];
				updateBins[0][1] = bin[1];
				//Calculating weight types
				//'0' offset
				weightTypes[0][0] = (float)(1.0 - abs((float)(a - (updateBins[0][0] * ratioA + ratioA / 2.0))) / ratioA);
				weightTypes[0][1] = (float)(1.0 - abs((float)(b - (updateBins[0][1] * ratioB + ratioB / 2.0))) / ratioB);
				//'+/- 1' offset
				weightTypes[1][0] = (float)(1.0 - abs((float)(a - ((bin[0] + valuePos[0]) * ratioA + ratioA / 2.0))) / ratioA);
				weightTypes[1][1] = (float)(1.0 - abs((float)(b - ((bin[1] + valuePos[1]) * ratioB + ratioB / 2.0))) / ratioB);
				
				binWeightCoeff[0][0] = weightTypes[0][0];
				binWeightCoeff[0][1] = weightTypes[0][1];
				bins2Update = 1;

				if (!((valuePos[0] == 0) && (valuePos[1] == 0)))	//current color is in bin centar
				{
					//first coordinate
					if (valuePos[0] != 0)
					{
						updateBins[bins2Update][0] = bin[0] + valuePos[0];
						if ((updateBins[bins2Update][0] >= 0) && (updateBins[bins2Update][0] < histBase[0]))
						{
							updateBins[bins2Update][1] = bin[1];
							
							binWeightCoeff[bins2Update][0] = weightTypes[abs(bin[0] - updateBins[bins2Update][0])][0];
							binWeightCoeff[bins2Update][1] = weightTypes[abs(bin[1] - updateBins[bins2Update][1])][1];
							bins2Update++;

							//first & second coordinate
							updateBins[bins2Update][1] = bin[1] + valuePos[1];
							if ((updateBins[bins2Update][1] >= 0) && (updateBins[bins2Update][1] < histBase[1]))
							{
								updateBins[bins2Update][0] = bin[0] + valuePos[0];
								binWeightCoeff[bins2Update][0] = weightTypes[abs(bin[0] - updateBins[bins2Update][0])][0];
								binWeightCoeff[bins2Update][1] = weightTypes[abs(bin[1] - updateBins[bins2Update][1])][1];
								bins2Update++;
							}
						}
					}

					//second coordinate
					if (valuePos[1] != 0)
					{
						updateBins[bins2Update][1] = bin[1] + valuePos[1];
						if ((updateBins[bins2Update][1] >= 0)  && (updateBins[bins2Update][1] < histBase[1]))
						{
							updateBins[bins2Update][0] = bin[0];
							binWeightCoeff[bins2Update][0] = weightTypes[abs(bin[0] - updateBins[bins2Update][0])][0];
							binWeightCoeff[bins2Update][1] = weightTypes[abs(bin[1] - updateBins[bins2Update][1])][1];
							bins2Update++;
						}
					}
				}

				//Updating all taged bins
				for (int i = 0; i < bins2Update; i++)
				{
					adr = (int)(updateBins[i][0] * histBase[1] + updateBins[i][1]);

					if (histLab[adr])
					{
						//azuriramo lokalni trikutic
						((RVLQLIST_HIST_ENTRY*)histLab[adr])->value += binWeightCoeff[i][0] * binWeightCoeff[i][1];
						//azuriramo objekt
						((RVLQLIST_HIST_ENTRY*)objHistLab[adr])->value += binWeightCoeff[i][0] * binWeightCoeff[i][1];
					}
					else
					{
						//stvaramo novi element koji ce biti i u arrayu i u povezanom popisu
						RVLQLIST_HIST_ENTRY *pHistEntryT;// = new RVLQLIST_HIST_ENTRY_SHORT[1];
						RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST_HIST_ENTRY, pHistEntryT);
						pHistEntryT->adr = adr;
						pHistEntryT->value = binWeightCoeff[i][0] * binWeightCoeff[i][1];
						histLab[adr] = pHistEntryT;
						RVLQLIST_ADD_ENTRY(histLabList, pHistEntryT);
						
						if (!objHistLab[adr])
						{
							RVLQLIST_HIST_ENTRY *pHistEntryO;// = new RVLQLIST_HIST_ENTRY_SHORT[1];
							RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST_HIST_ENTRY, pHistEntryO);
							pHistEntryO->adr = adr;
							pHistEntryO->value = binWeightCoeff[i][0] * binWeightCoeff[i][1];
							objHistLab[adr] = pHistEntryO;
							RVLQLIST_ADD_ENTRY(objHistLabList, pHistEntryO);
						}
						else
							((RVLQLIST_HIST_ENTRY*)objHistLab[adr])->value += binWeightCoeff[i][0] * binWeightCoeff[i][1];	//azuriramo objekt
					}

				}
			}		
		}
		//Stvaramo i sortiramo RGB histogram za trokut
		pTriangle->m_histRGB = InsertSortQLISTWithCopy<RVLQLIST_HIST_ENTRY>(histLabList, RVLQLIST_DESCENDING, this->m_pClass->m_pMem0, TRUE);
		pElement = (RVLQLIST_PTR_ENTRY*)pElement->pNext;
	}
	//Stvaramo i sortiramo RGB histogram za cijeli objekt
	this->m_histRGB = InsertSortQLISTWithCopy<RVLQLIST_HIST_ENTRY>(objHistLabList, RVLQLIST_DESCENDING, this->m_pClass->m_pMem0, TRUE);
	this->m_histRGB_base[0] = histBase[0];
	this->m_histRGB_base[1] = histBase[1];
	this->m_histRGB_base[2] = histBase[2];
	this->m_ColorSystem = RVL_MESH_COLOR_CIELAB_2D;

	//Azuriramo histogram od parenta ako postoji(NETESTIRANO!!!! STO AKO IMAJU RAZLICITE BAZE?!?!?)
	if (((CRVL3DMeshObject*)this->parentMeshObject)->m_histRGB)
	{
		RVLQLIST *pParentHistList = ((CRVL3DMeshObject*)this->parentMeshObject)->m_histRGB;
		if (pParentHistList->pFirst)
		{
			RVLQLIST_HIST_ENTRY *pParentHistEntry = (RVLQLIST_HIST_ENTRY *)pParentHistList->pFirst;
			RVLQLIST_HIST_ENTRY *pObjHistEntry = (RVLQLIST_HIST_ENTRY *)this->m_histRGB->pFirst;
			bool insert = FALSE;
			while(pObjHistEntry)
			{
				while (pParentHistEntry)
				{
					if (pObjHistEntry->adr == pParentHistEntry->adr)
						pParentHistEntry->value += pObjHistEntry->value;
					pParentHistEntry = (RVLQLIST_HIST_ENTRY *)pParentHistEntry->pNext;
					if (!pParentHistEntry)
					{
						RVLQLIST_ADD_ENTRY(pParentHistList, pObjHistEntry);
						insert = TRUE;
						break;
					}
				}
				pObjHistEntry = (RVLQLIST_HIST_ENTRY *)pObjHistEntry->pNext;
			}
			if (insert)
				((CRVL3DMeshObject*)this->parentMeshObject)->m_histRGB = InsertSortQLISTWithCopy<RVLQLIST_HIST_ENTRY>(pParentHistList, RVLQLIST_DESCENDING, this->m_pClass->m_pMem0, TRUE);

		}
		else
		{
			//Kopiraj podatke od objekta
			RVLQLIST_HIST_ENTRY *pObjHistEntry = (RVLQLIST_HIST_ENTRY *)this->m_histRGB->pFirst;
			while(pObjHistEntry)
			{
				RVLQLIST_HIST_ENTRY *pParentHistNewEntry;// = new RVLQLIST_HIST_ENTRY_SHORT[1];
				RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem0, RVLQLIST_HIST_ENTRY, pParentHistNewEntry);
				pParentHistNewEntry->adr = pObjHistEntry->adr;
				pParentHistNewEntry->value = pObjHistEntry->value;
				RVLQLIST_ADD_ENTRY(pParentHistList, pParentHistNewEntry);

				pObjHistEntry = (RVLQLIST_HIST_ENTRY *)pObjHistEntry->pNext;
			}
		}
	}
}

void CRVL3DMeshObject::RVLCalculateXYZHist(IplImage* xyzImg, float* histBase, bool soft)
{
	this->RVLCalculateRGBHist(xyzImg, histBase, soft);
}

void CRVL3DMeshObject::RVLCalculateRGCHist(IplImage* rgbImg, float* histBase, bool soft)
{
	int noBins = (int)histBase[0] * histBase[1];
	float ratioR = 1.0 / histBase[0]; //how many levels are per bin
	float ratioG = 1.0 / histBase[1]; //how many levels are per bin
	CRVL2DRegion2 *pTriangle;
	RVL3DPOINT2 **ppPt, **pPtArrayEnd;
	RVL3DPOINT2 *pPt;
	int iPix;
	int label;
	int r = 0, g = 0, b = 0, rgb_x = 0, rgb_y = 0;
	float rr = 0.0, gg = 0.0;
	int w = rgbImg->width;
	int h = rgbImg->height;
	int adr;

	int updateBins[4][2];	//Needed for soft histograms, coordinates for updated bins
	memset(updateBins, 0, 4 * 2 * sizeof(int));
	int bins2Update = 0;	//Needed for soft histograms, number of bins needed to update
	float binCentar[2];	//Needed for soft histograms, closest bin centar
	int bin[2];	//Needed for soft histograms, central bin coordinates
	int valuePos[2];	//current value above, below or on bin centar, per dimension(r,g)
	float binWeightCoeff[4][2];	//Needed for soft histograms, calculated coefficients for faster analysis
	float weightTypes[2][2];	//Needed for soft histograms, there can be only two weights, offset by 0 or +/- 1
	memset(binWeightCoeff, 0, 4 * 2 * sizeof(float));

	void** histRGB;// = new void*[noBins];
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, void*, noBins, histRGB);
	RVLQLIST *histRGBList;// = new RVLQLIST[1];
	RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST, histRGBList);

	void** objHistRGB;// = new void*[noBins];
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, void*, noBins, objHistRGB);
	for(int i = 0; i < noBins; i++)
	{
		if (objHistRGB[i])
			objHistRGB[i] = NULL;
	}
	RVLQLIST *objHistRGBList;// = new RVLQLIST[1];
	RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST, objHistRGBList);
	RVLQLIST_INIT(objHistRGBList);

	uchar *ptr;
	RVLQLIST_PTR_ENTRY *pElement;
	pElement = (RVLQLIST_PTR_ENTRY*)this->m_FaceList->pFirst;

	while(pElement)
	{
		pTriangle = (CRVL2DRegion2 *)(pElement->Ptr);

		if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		label = pTriangle->m_Label;

		pPtArrayEnd = pTriangle->m_pPoint3DArray + pTriangle->m_n3DPts;

		for(int i = 0; i < noBins; i++)
		{
			if (histRGB[i])
				histRGB[i] = NULL;
		}
		RVLQLIST_INIT(histRGBList);	//resetiramo listu 

		//pTriangle->m_histRGB;// = new RVLQLIST[1];
		RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST, pTriangle->m_histRGB);
		RVLQLIST_INIT(pTriangle->m_histRGB);
		pTriangle->m_histRGB_base[0] = histBase[0];
		pTriangle->m_histRGB_base[1] = histBase[1];
		pTriangle->m_histRGB_base[2] = histBase[2];
		pTriangle->m_ColorSystem = RVL_MESH_COLOR_RGB_N_2D;

		for(ppPt = pTriangle->m_pPoint3DArray; ppPt < pPtArrayEnd; ppPt++)
		{
			pPt = *ppPt;

			iPix = pPt->iPix;
			//ako je maska postavljena i nije dozvoljen idemo dalje
			if (pTriangle->m_uPixMask && (pTriangle->m_uPixMask[iPix] == 1))
				continue;
			//Povecavamo broj koristenih piksela s bojom
			this->m_noUsedColorPts++;
			//Provjeriti
			rgb_y = pPt->iPixRGB / w;
			rgb_x = pPt->iPixRGB - rgb_y * w;
			if (rgb_y != 0)
				rgb_x --;
			ptr = (uchar*)(rgbImg->imageData + rgb_y * rgbImg->widthStep);
			if (rgbImg->channelSeq[0] == 'B')
			{
				b = ptr[rgb_x * 3];
				g = ptr[rgb_x * 3 + 1];
				r = ptr[rgb_x * 3 + 2];
			}
			else
			{
				r = ptr[rgb_x * 3];
				g = ptr[rgb_x * 3 + 1];
				b = ptr[rgb_x * 3 + 2];
			}

			if ((r == 0) && (g == 0) && (b == 0))
			{
				rr = 0.0;
				gg = 0.0;
			}
			else
			{
				rr = (float)r / (float)(r + g + b);
				gg = (float)g / (float)(r + g + b);
			}

			if(!soft)
			{
				adr = (int)(floor(rr / ratioR) * histBase[1] + floor(gg / ratioG)); // i = rr/(256/16) * 16 + gg/(256/16)
				if (histRGB[adr])
				{
					//azuriramo lokalni trikutic
					((RVLQLIST_HIST_ENTRY*)histRGB[adr])->value++;
					//azuriramo objekt
					((RVLQLIST_HIST_ENTRY*)objHistRGB[adr])->value++;
				}
				else
				{
					//stvaramo novi element koji ce biti i u arrayu i u povezanom popisu
					RVLQLIST_HIST_ENTRY *pHistEntryT;// = new RVLQLIST_HIST_ENTRY_SHORT[1];
					RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST_HIST_ENTRY, pHistEntryT);
					pHistEntryT->adr = adr;
					pHistEntryT->value = 1;
					histRGB[adr] = pHistEntryT;
					RVLQLIST_ADD_ENTRY(histRGBList, pHistEntryT);
					
					if (!objHistRGB[adr])
					{
						RVLQLIST_HIST_ENTRY *pHistEntryO;// = new RVLQLIST_HIST_ENTRY_SHORT[1];
						RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST_HIST_ENTRY, pHistEntryO);
						pHistEntryO->adr = adr;
						pHistEntryO->value = 1;
						objHistRGB[adr] = pHistEntryO;
						RVLQLIST_ADD_ENTRY(objHistRGBList, pHistEntryO);
					}
					else
						((RVLQLIST_HIST_ENTRY*)objHistRGB[adr])->value++;	//azuriramo objekt
				}
			}
			else
			{
				//closest bin centar
				bin[0] = (int)floor(rr / ratioR);
				bin[1] = (int)floor(gg / ratioG);
				binCentar[0] = (float)(bin[0] * ratioR + ratioR / 2);
				binCentar[1] = (float)(bin[1] * ratioG + ratioG / 2);
				
				//is current value above bin centar per dimension
				if (rr > binCentar[0])
					valuePos[0] = 1;
				else if (rr == binCentar[0])
					valuePos[0] = 0;
				else if (rr < binCentar[0])
					valuePos[0] = -1;

				if (gg > binCentar[1])
					valuePos[1] = 1;
				else if (gg == binCentar[1])
					valuePos[1] = 0;
				else if (gg < binCentar[1])
					valuePos[1] = -1;

				//Finding which bin to update
				updateBins[0][0] = bin[0];
				updateBins[0][1] = bin[1];
				//Calculating weight types
				//'0' offset
				weightTypes[0][0] = (float)(1.0 - abs((float)(rr - (updateBins[0][0] * ratioR + ratioR / 2.0))) / ratioR);
				weightTypes[0][1] = (float)(1.0 - abs((float)(gg - (updateBins[0][1] * ratioG + ratioG / 2.0))) / ratioG);
				//'+/- 1' offset
				weightTypes[1][0] = (float)(1.0 - abs((float)(rr - ((bin[0] + valuePos[0]) * ratioR + ratioR / 2.0))) / ratioR);
				weightTypes[1][1] = (float)(1.0 - abs((float)(gg - ((bin[1] + valuePos[1]) * ratioG + ratioG / 2.0))) / ratioG);
				
				binWeightCoeff[0][0] = weightTypes[0][0];
				binWeightCoeff[0][1] = weightTypes[0][1];
				bins2Update = 1;

				if (!((valuePos[0] == 0) && (valuePos[1] == 0)))	//current color is in bin centar
				{
					//first coordinate
					if (valuePos[0] != 0)
					{
						updateBins[bins2Update][0] = bin[0] + valuePos[0];
						if ((updateBins[bins2Update][0] >= 0) && (updateBins[bins2Update][0] < histBase[0]))
						{
							updateBins[bins2Update][1] = bin[1];
							
							binWeightCoeff[bins2Update][0] = weightTypes[abs(bin[0] - updateBins[bins2Update][0])][0];
							binWeightCoeff[bins2Update][1] = weightTypes[abs(bin[1] - updateBins[bins2Update][1])][1];
							bins2Update++;

							//first & second coordinate
							updateBins[bins2Update][1] = bin[1] + valuePos[1];
							if ((updateBins[bins2Update][1] >= 0) && (updateBins[bins2Update][1] < histBase[1]))
							{
								updateBins[bins2Update][0] = bin[0] + valuePos[0];
								binWeightCoeff[bins2Update][0] = weightTypes[abs(bin[0] - updateBins[bins2Update][0])][0];
								binWeightCoeff[bins2Update][1] = weightTypes[abs(bin[1] - updateBins[bins2Update][1])][1];
								bins2Update++;
							}
						}
					}

					//second coordinate
					if (valuePos[1] != 0)
					{
						updateBins[bins2Update][1] = bin[1] + valuePos[1];
						if ((updateBins[bins2Update][1] >= 0)  && (updateBins[bins2Update][1] < histBase[1]))
						{
							updateBins[bins2Update][0] = bin[0];
							binWeightCoeff[bins2Update][0] = weightTypes[abs(bin[0] - updateBins[bins2Update][0])][0];
							binWeightCoeff[bins2Update][1] = weightTypes[abs(bin[1] - updateBins[bins2Update][1])][1];
							bins2Update++;
						}
					}
				}

				//Updating all taged bins
				for (int i = 0; i < bins2Update; i++)
				{
					adr = (int)(updateBins[i][0] * histBase[1] + updateBins[i][1]);

					if (histRGB[adr])
					{
						//azuriramo lokalni trokutic
						((RVLQLIST_HIST_ENTRY*)histRGB[adr])->value += binWeightCoeff[i][0] * binWeightCoeff[i][1];
						//azuriramo objekt
						((RVLQLIST_HIST_ENTRY*)objHistRGB[adr])->value += binWeightCoeff[i][0] * binWeightCoeff[i][1];
					}
					else
					{
						//stvaramo novi element koji ce biti i u arrayu i u povezanom popisu
						RVLQLIST_HIST_ENTRY *pHistEntryT;// = new RVLQLIST_HIST_ENTRY_SHORT[1];
						RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST_HIST_ENTRY, pHistEntryT);
						pHistEntryT->adr = adr;
						pHistEntryT->value = binWeightCoeff[i][0] * binWeightCoeff[i][1];
						histRGB[adr] = pHistEntryT;
						RVLQLIST_ADD_ENTRY(histRGBList, pHistEntryT);
						
						if (!objHistRGB[adr])
						{
							RVLQLIST_HIST_ENTRY *pHistEntryO;// = new RVLQLIST_HIST_ENTRY_SHORT[1];
							RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST_HIST_ENTRY, pHistEntryO);
							pHistEntryO->adr = adr;
							pHistEntryO->value = binWeightCoeff[i][0] * binWeightCoeff[i][1];
							objHistRGB[adr] = pHistEntryO;
							RVLQLIST_ADD_ENTRY(objHistRGBList, pHistEntryO);
						}
						else
							((RVLQLIST_HIST_ENTRY*)objHistRGB[adr])->value += binWeightCoeff[i][0] * binWeightCoeff[i][1];	//azuriramo objekt
					}

				}

			}
			
		}
		//Stvaramo i sortiramo RGB histogram za trokut
		pTriangle->m_histRGB = InsertSortQLISTWithCopy<RVLQLIST_HIST_ENTRY>(histRGBList, RVLQLIST_DESCENDING, this->m_pClass->m_pMem0, TRUE);
		pElement = (RVLQLIST_PTR_ENTRY*)pElement->pNext;
	}
	//Stvaramo i sortiramo RGB histogram za cijeli objekt
	this->m_histRGB = InsertSortQLISTWithCopy<RVLQLIST_HIST_ENTRY>(objHistRGBList, RVLQLIST_DESCENDING, this->m_pClass->m_pMem0, TRUE);
	this->m_histRGB_base[0] = histBase[0];
	this->m_histRGB_base[1] = histBase[1];
	this->m_histRGB_base[2] = histBase[2];
	this->m_ColorSystem = RVL_MESH_COLOR_RGB_N_2D;

	//Azuriramo histogram od parenta ako postoji(NETESTIRANO!!!! STO AKO IMAJU RAZLICITE BAZE?!?!?)
	if (((CRVL3DMeshObject*)this->parentMeshObject)->m_histRGB)
	{
		RVLQLIST *pParentHistList = ((CRVL3DMeshObject*)this->parentMeshObject)->m_histRGB;
		if (pParentHistList->pFirst)
		{
			RVLQLIST_HIST_ENTRY *pParentHistEntry = (RVLQLIST_HIST_ENTRY *)pParentHistList->pFirst;
			RVLQLIST_HIST_ENTRY *pObjHistEntry = (RVLQLIST_HIST_ENTRY *)this->m_histRGB->pFirst;
			bool insert = FALSE;
			while(pObjHistEntry)
			{
				while (pParentHistEntry)
				{
					if (pObjHistEntry->adr == pParentHistEntry->adr)
						pParentHistEntry->value += pObjHistEntry->value;
					pParentHistEntry = (RVLQLIST_HIST_ENTRY *)pParentHistEntry->pNext;
					if (!pParentHistEntry)
					{
						RVLQLIST_ADD_ENTRY(pParentHistList, pObjHistEntry);
						insert = TRUE;
						break;
					}
				}
				pObjHistEntry = (RVLQLIST_HIST_ENTRY *)pObjHistEntry->pNext;
			}
			if (insert)
				((CRVL3DMeshObject*)this->parentMeshObject)->m_histRGB = InsertSortQLISTWithCopy<RVLQLIST_HIST_ENTRY>(pParentHistList, RVLQLIST_DESCENDING, this->m_pClass->m_pMem0, TRUE);

		}
		else
		{
			//Kopiraj podatke od objekta
			RVLQLIST_HIST_ENTRY *pObjHistEntry = (RVLQLIST_HIST_ENTRY *)this->m_histRGB->pFirst;
			while(pObjHistEntry)
			{
				RVLQLIST_HIST_ENTRY *pParentHistNewEntry;// = new RVLQLIST_HIST_ENTRY_SHORT[1];
				RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem0, RVLQLIST_HIST_ENTRY, pParentHistNewEntry);
				pParentHistNewEntry->adr = pObjHistEntry->adr;
				pParentHistNewEntry->value = pObjHistEntry->value;
				RVLQLIST_ADD_ENTRY(pParentHistList, pParentHistNewEntry);

				pObjHistEntry = (RVLQLIST_HIST_ENTRY *)pObjHistEntry->pNext;
			}
		}
	}	
}

void CRVL3DMeshObject::SaveRGBDSegmentMasks(char* rgbMaskPath,
											char* depthMaskPath,
											RVL3DPOINT2 **Point3DMap,
											int width,
											int height)
{
	RVLQLIST *pMeshObjList;
	RVLQLIST_PTR_ENTRY *pElement;
	RVLQLIST_PTR_ENTRY *pObject;
	CRVL3DMeshObject *tempObj;
	if (this->m_FaceList)
	{
		pMeshObjList = new RVLQLIST[1];
		RVLQLIST_INIT(pMeshObjList);
		RVLQLIST_PTR_ENTRY *pObj = new RVLQLIST_PTR_ENTRY[1];
		pObj->Ptr = this;
		RVLQLIST_ADD_ENTRY(pMeshObjList, pObj);
	}
	else if (this->m_ChildMeshObjects)
	{
		pMeshObjList = this->m_ChildMeshObjects;
	}

	int iPt = 0;

	CRVL2DRegion2 *pTriangle;

	int iPix;
	RVL3DPOINT2 **ppPt, **pPtArrayEnd;
	RVL3DPOINT2 *pPt;

	int *rgbMask = new int[width * height];
	memset(rgbMask, 0, width * height * sizeof(int));
	int *depthMask = new int[width * height];
	memset(depthMask, 0, width * height * sizeof(int));
	pObject = (RVLQLIST_PTR_ENTRY*)pMeshObjList->pFirst;
	int br = 1;
	while (pObject)
	{
		tempObj = (CRVL3DMeshObject*)pObject->Ptr;

		pElement = (RVLQLIST_PTR_ENTRY*)tempObj->m_FaceList->pFirst;
		while(pElement)
		{
			pTriangle = (CRVL2DRegion2 *)(pElement->Ptr);

			if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
				continue;

			pPtArrayEnd = pTriangle->m_pPoint3DArray + pTriangle->m_n3DPts;

			for(ppPt = pTriangle->m_pPoint3DArray; ppPt < pPtArrayEnd; ppPt++)
			{
				pPt = *ppPt;

				iPix = pPt->iPix;
				//ako je maska postavljena i nije dozvoljen pixel idemo dalje
				if (pTriangle->m_uPixMask && (pTriangle->m_uPixMask[iPix] == 1))
					continue;
				
				depthMask[iPix] = br;
				rgbMask[pPt->iPixRGB] = br;
			}
			pElement = (RVLQLIST_PTR_ENTRY*)pElement->pNext;
		}
		pObject = (RVLQLIST_PTR_ENTRY*)pObject->pNext;
		br++;
	}

	pObject = (RVLQLIST_PTR_ENTRY*)pMeshObjList->pFirst;
	while (pObject)
	{
		tempObj = (CRVL3DMeshObject*)pObject->Ptr;

		pElement = (RVLQLIST_PTR_ENTRY*)tempObj->m_FaceList->pFirst;
		while(pElement)
		{
			pTriangle = (CRVL2DRegion2 *)(pElement->Ptr);

			if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
				continue;

			pPtArrayEnd = pTriangle->m_pPoint3DArray + pTriangle->m_n3DPts;

			for(ppPt = pTriangle->m_pPoint3DArray; ppPt < pPtArrayEnd; ppPt++)
			{
				pPt = *ppPt;

				iPix = pPt->iPix;
				if (pTriangle->m_uPixMask && (pTriangle->m_uPixMask[iPix] == 1))
					rgbMask[pPt->iPixRGB] = 0;
			}
			pElement = (RVLQLIST_PTR_ENTRY*)pElement->pNext;
		}
		pObject = (RVLQLIST_PTR_ENTRY*)pObject->pNext;
	}

	ofstream rgbFileStream;
	ofstream depthFileStream;
	rgbFileStream.open(rgbMaskPath, ofstream::out);
	depthFileStream.open(depthMaskPath, ofstream::out);
	for(int i = 0; i < width * height; i++)
	{
		rgbFileStream << rgbMask[i] << " ";
		depthFileStream << depthMask[i] << " ";
		if (((i + 1) % width == 0) && (i + 1 != width * height))
		{
			rgbFileStream << "\n";
			depthFileStream << "\n";
		}
	}
	rgbFileStream.close();
	depthFileStream.close();

	delete [] rgbMask;
	delete [] depthMask;
}

float CRVL3DMeshObject::RVLCupecRGBProbMeasure(CRVL3DMeshObject *model, CRVL3DMeshObject *scene, int usedBins)
{
	//ako nepostoje odgovarajuci histogrami ili za nihovo stvaranje nije koristen ni jedan piksel u boji prekini
	if (!scene->m_histRGB || !model->m_histRGB || scene->m_noUsedColorPts == 0 || model->m_noUsedColorPts == 0)
		return 0.0;
	//provjera da li su im baze histograma iste(NAPRAVITI DA SE REDUCIRA ONAJ HISTOGRAM KOJI JE PREVELIK!!!!)
	//takoder provjera da li su isti color sustavi koristeni
	if ((scene->m_histRGB_base[0] != model->m_histRGB_base[0]) || (scene->m_ColorSystem != model->m_ColorSystem))
		return -1.0;

	//Ne radi dobro ako histogram nije 3D!!!!
	if (usedBins == -1)
		usedBins = scene->m_histRGB_base[0] * scene->m_histRGB_base[1] * scene->m_histRGB_base[2];
	float intersectValue = 0.0;
	float *objValues;// = new float[usedBins];
	RVLMEM_ALLOC_STRUCT_ARRAY(scene->m_pClass->m_pMem2, float, usedBins, objValues);
	// new code (CUPEC)

#ifdef RVL_MESH_INTERSECT_COLOR_HISTOGRAM_DEBUG_LOG
	FILE *fp;

	fopen_s(&fp, "C:\\RVL\\Debug\\ColorHistogramM.dat", "w");
#endif

	int *ColorData;

	RVLMEM_ALLOC_STRUCT_ARRAY(model->m_pClass->m_pMem2, int, 3 * usedBins, ColorData);

	int foundObjBinsModel = 0;
	int *Cm = ColorData;
	DWORD adr;
	int base = (DOUBLE2INT(model->m_histRGB_base[0]) >> 1);
	int Shiftm = 0;

	while(base)
	{
		Shiftm++;
		base = (base >> 1);
	}

	double fnPts = 0.0;

	int i;

	RVLQLIST_HIST_ENTRY *pHistEntry = (RVLQLIST_HIST_ENTRY*)model->m_histRGB->pFirst;

	for (i = 0; i < usedBins; i++)
	{
		if (!pHistEntry)
			break;

		fnPts += pHistEntry->value;

		pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
	}

	pHistEntry = (RVLQLIST_HIST_ENTRY*)model->m_histRGB->pFirst;

	for (i = 0; i < usedBins; i++)
	{
		objValues[i] = pHistEntry->value / fnPts;
		adr = pHistEntry->adr;
		*(Cm++) = ((adr & 0x00000007) << 1) + 1;
		adr = (adr >> Shiftm);
		*(Cm++) = ((adr & 0x00000007) << 1) + 1;
		adr = (adr >> Shiftm);
		*(Cm++) = ((adr & 0x00000007) << 1) + 1;
		foundObjBinsModel++;
		pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
		if (!pHistEntry)
			break;
#ifdef RVL_MESH_INTERSECT_COLOR_HISTOGRAM_DEBUG_LOG
		fprintf(fp, "%d\t%d\t%d\t%lf\n", ColorData[3 * i + 0], ColorData[3 * i + 1], ColorData[3 * i + 2], objValues[i]);
#endif
	}

#ifdef RVL_MESH_INTERSECT_COLOR_HISTOGRAM_DEBUG_LOG
		fclose(fp);

		fopen_s(&fp, "C:\\RVL\\Debug\\ColorHistogramS.dat", "w");

		FILE *fpTable;

		fopen_s(&fpTable, "C:\\RVL\\Debug\\ColorMatchTable.dat", "w");
#endif

	// parameters

	double sigman = 1.0 * 1.0;
	double sigmas = 0.3 * 0.3;
	double PClear = 0.9;

	/////

	double k1 = 2.0 * PI;
	double k7 = k1 * k1 * k1;
	double k2 = 2.0 * sigman * sigman;
	double k3 = sigmas / sigman;
	double k4 = 4.0 * k2 * sigmas;
	double k5 = 4.0 * k2 * sigman;
	double k6 = 1.0 / (2.0 * scene->m_histRGB_base[0]);
	double k8 = (1.0 - PClear) * k6;
	double k9 = k8 * k6 * k6;
	
	fnPts = 0.0;

	int foundObjBins = 0;

	pHistEntry = (RVLQLIST_HIST_ENTRY*)scene->m_histRGB->pFirst;

	while(pHistEntry)
	{
		if(foundObjBins >= usedBins)
			break;

		fnPts += pHistEntry->value;

		foundObjBins++;

		pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
	}
	
	pHistEntry = (RVLQLIST_HIST_ENTRY*)scene->m_histRGB->pFirst;

	float w;
	int C[3];
	int tmp3x1[3], eC[3];
	double det, lenCm2;
	double pC, pCifCm;
	double e1, e2, z1, z2;
	int j;

	for(i = 0; i < foundObjBins; i++)
	{
		w = 100.0 * pHistEntry->value / fnPts;
		adr = pHistEntry->adr;
		C[0] = ((adr & 0x00000007) << 1) + 1;
		adr = (adr >> Shiftm);
		C[1] = ((adr & 0x00000007) << 1) + 1;
		adr = (adr >> Shiftm);
		C[2] = ((adr & 0x00000007) << 1) + 1;

#ifdef RVL_MESH_INTERSECT_COLOR_HISTOGRAM_DEBUG_LOG
		fprintf(fp, "%d\t%d\t%d\t%lf\n", C[0], C[1], C[2], w);
#endif

		pC = k9;	// 3D PDF
		//pC = k8;	// 1D PDF

		for (j = 0; j < foundObjBinsModel; j++)
		{
			Cm = ColorData + 3 * j;

			lenCm2 = (double)RVLDOTPRODUCT3(Cm, Cm);

			det = k4 * lenCm2 + k5;  

			RVLCROSSPRODUCT3(Cm, C, tmp3x1)
			RVLDIF3VECTORS(C, Cm, eC)				// 3D PDF
			//RVLSUM3VECTORS(C, Cm, eC)				// 1D PDF

			e1 = (double)RVLDOTPRODUCT3(tmp3x1, tmp3x1);
			e2 = (double)RVLDOTPRODUCT3(eC, eC);
			
			pCifCm = 1/sqrt(k7 * det) * exp(-k2/det*(k3*e1 + e2));	// 3D PDF
			//pCifCm = 1/sqrt(k1 * sigman) * exp(-e1/e2/sigman);	// 1D PDF

#ifdef RVL_MESH_INTERSECT_COLOR_HISTOGRAM_DEBUG_LOG
			fprintf(fpTable, "%lf\t", pCifCm);
#endif

			pC += (pCifCm * objValues[j] * PClear);
		}

#ifdef RVL_MESH_INTERSECT_COLOR_HISTOGRAM_DEBUG_LOG
		fprintf(fpTable, "\n");
#endif
		z1 = lnFactorial_LookUpTable[DOUBLE2INT(w)];	// 3D PDF, histogram probability
		z2 = w * log(8.0 * pC);

		intersectValue += (z1 - z2);	// histogram probability

		//intersectValue -= z2;			// appearance probability

		pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
	}

#ifdef RVL_MESH_INTERSECT_COLOR_HISTOGRAM_DEBUG_LOG
	fclose(fp);

	fclose(fpTable);
#endif

	/////

	return intersectValue;
	//return -1.0; 
}

void CRVL3DMeshObject::CalculateTextonHistogram(float lambC1, float lambC2, float lambCap, int lambBins, int angBins)
{
	if (!this->m_TextonList.pFirst)	//ako ne postoji lista textona prekini
		return;

	int noBins = lambBins * (angBins + 1);	//angle noBins is increased because of classifiying textons who got lambda2/lambda1 < 2 and for lambda1 > lambC1
	float lambRatio = lambCap / (float)lambBins;
	float angRatio = (PI / 2.0) / angBins;

	void** textonHist;
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, void*, noBins, textonHist);
	RVLQLIST *textonHistList;
	RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST, textonHistList);
	RVLQLIST_INIT(textonHistList);
	for(int i = 0; i < noBins; i++)
	{
		if (textonHist[i])
			textonHist[i] = NULL;
	}

	int adr = 0;
	float orientation = 0.0;
	float noPix = 0.0;
	RVLTEXTON *pElement;
	pElement = (RVLTEXTON*)this->m_TextonList.pFirst;

	while(pElement)
	{
		//Prolazimo sve tekstone i stvaramo histogram
		if (pElement->Size > lambC1)
		{
			pElement = (RVLTEXTON*)pElement->pNext;
			continue;
		}
		//if (pElement->Size > lambC1)
		//	adr = (int)(lambBins * (angBins + 1) - 1);	//The very last bin
		if ((pElement->Elongation < lambC2) && (pElement->Size < lambC1))
			adr = (int)(floor(pElement->Size / lambRatio) * (angBins + 1) + (angBins + 1) - 1); //last bin // second to last column
		else if ((pElement->Elongation >= lambC2) && (pElement->Size < lambC1))
		{
			if (pElement->Orientation < 0)
				orientation = pElement->Orientation + PI / 2.0;
			else
				orientation = pElement->Orientation;
			adr = (int)(floor(pElement->Size / lambRatio) * (angBins + 1) + floor(orientation / angRatio));//lambBins * angBins histogram
		}
		/*else
			adr = angBins + 1 - 1;*/
		
		
		noPix += pElement->nPts;	//Ukupni broj svih tocaka ukljucenih, za normiranje

		if (textonHist[adr])
		{
			//azuriramo objekt
			((RVLQLIST_HIST_ENTRY*)textonHist[adr])->value += pElement->nPts;
		}
		else
		{
			//stvaramo novi element koji ce biti i u arrayu i u povezanom popisu
			RVLQLIST_HIST_ENTRY *pHistEntryT;
			RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST_HIST_ENTRY, pHistEntryT);
			pHistEntryT->adr = adr;
			pHistEntryT->value = pElement->nPts;
			textonHist[adr] = pHistEntryT;
			RVLQLIST_ADD_ENTRY(textonHistList, pHistEntryT);
		}
		pElement = (RVLTEXTON*)pElement->pNext;
	}
	//Normiranje
	for (int i = 0; i < noBins; i++)
	{
		if (textonHist[i] && noPix > 0)
			((RVLQLIST_HIST_ENTRY*)textonHist[i])->value = ((RVLQLIST_HIST_ENTRY*)textonHist[i])->value / noPix;
	}
	//Sortiranje
	this->m_TextonHist = InsertSortQLISTWithCopy<RVLQLIST_HIST_ENTRY>(textonHistList, RVLQLIST_DESCENDING, this->m_pClass->m_pMem0, TRUE);

	//Popunjavamo listu histograma i azuriramo roditeljski ako postoji (NIJE IMPLEMENTIRANO)
	//if (((CRVL3DMeshObject*)this->parentMeshObject)->m_TextonHist)
	//{
	//	RVLQLIST_HIST_ENTRY *pEntryT;
	//	RVLQLIST_HIST_ENTRY *pEntryP;
	//	RVLQLIST *parentTextonHist;
	//	parentTextonHist = (RVLQLIST*)((CRVL3DMeshObject*)this->parentMeshObject)->m_TextonHist;
	//	pEntryT = (RVLQLIST_HIST_ENTRY*)(parentTextonHist)->pFirst;
	//	pEntryP = pEntryT;
	//	
	//	for(int i = 0; i < lambBins * (angBins + 2); i++)
	//	{
	//		if (lbpRiuVarHistObject[i] != 0)
	//		{
	//			RVLQLIST_HIST_ENTRY *pHistEntry;
	//			RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST_HIST_ENTRY, pHistEntry);
	//			pHistEntry->adr = i;
	//			pHistEntry->value = lbpRiuVarHistObject[i];
	//			RVLQLIST_ADD_ENTRY(this->m_LBP_RIU_VAR, pHistEntry);
	//			
	//			//Provjeravamo s parentom
	//			while (pEntryT->adr < i)
	//			{
	//				pEntryP = pEntryT;
	//				pEntryT = (RVLQLIST_HIST_ENTRY*)pEntryT->pNext;
	//			}
	//			if (pEntryT->adr = i)
	//				pEntryT->value += lbpRiuVarHistObject[i];
	//			else if (pEntryT->adr > i)
	//			{
	//				RVLQLIST_HIST_ENTRY *pHistNewEntry;
	//				RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST_HIST_ENTRY, pHistNewEntry);
	//				pHistNewEntry->adr = i;
	//				pHistNewEntry->value = lbpRiuVarHistObject[i];
	//				RVLQLIST_INSERT_ENTRY(parentLBPRiuVar, pEntryP, pEntryT, pHistNewEntry);
	//			}

	//		}
	//	}				
	//}
	//else
	//{
	//	for(int i = 0; i < hist_bins_lbp * hist_bins_var; i++)
	//	{
	//		if (lbpRiuVarHistObject[i] != 0)
	//		{
	//			RVLQLIST_HIST_ENTRY *pHistEntry;
	//			RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem, RVLQLIST_HIST_ENTRY, pHistEntry);
	//			pHistEntry->adr = i;
	//			pHistEntry->value = lbpRiuVarHistObject[i];
	//			RVLQLIST_ADD_ENTRY(this->m_LBP_RIU_VAR, pHistEntry);
	//		}
	//	}
	//}
}

float CRVL3DMeshObject::CalculateTextonHistogram2(float lambC,
												float elongC,
												float lambCap,
												float elongCap,
												int lambBins,
												int elongBins)
{
	if (!this->m_TextonList.pFirst)	//ako ne postoji lista textona prekini
		return -1.0;
	
	float lambRatio = lambCap / (float)lambBins;
	float elongRatio = elongCap / (float)elongBins;
	int shift = floor(1.0 / elongRatio);
	int noBins = lambBins * (elongBins - shift);

	void** textonHist;
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, void*, noBins, textonHist);
	RVLQLIST *textonHistList;
	RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST, textonHistList);
	RVLQLIST_INIT(textonHistList);
	for(int i = 0; i < noBins; i++)
	{
		if (textonHist[i])
			textonHist[i] = NULL;
	}

	int adr = 0;
	float noPix = 0.0;
	int noTex = 0;
	RVLTEXTON *pElement;
	pElement = (RVLTEXTON*)this->m_TextonList.pFirst;

	while(pElement)
	{
		//Prolazimo sve tekstone i stvaramo histogram
		if ((pElement->Size > lambC) || ((pElement->Elongation > elongC)))
		{
			pElement = (RVLTEXTON*)pElement->pNext;
			continue;
		}
		//if (pElement->Size > lambC)
		//	adr = (int)(lambBins * elongBins - 1);	//The very last bin
		//else if (pElement->Elongation > elongC)
		//	adr = (int)((lambBins - 1) * elongBins - 1);	//row above very last bin
		//else
		//	adr = (int)(floor(pElement->Size / lambRatio) * elongBins + floor(pElement->Elongation / elongRatio)) - 1; //'-1' shift because lambda2/lambda1 cannot be less than 1
		//
		if ((pElement->Size < lambC) && (pElement->Elongation < elongC))
		{
			adr = (int)(floor(pElement->Size / lambRatio) * (elongBins - shift) + floor(pElement->Elongation / elongRatio)) - shift; //'-1' shift because lambda2/lambda1 cannot be less than 1
			noPix += pElement->nPts;	//Ukupni broj svih tocaka ukljucenih, za normiranje
		}

		if (textonHist[adr])
		{
			//azuriramo objekt
			((RVLQLIST_HIST_ENTRY*)textonHist[adr])->value += pElement->nPts;
		}
		else
		{
			//stvaramo novi element koji ce biti i u arrayu i u povezanom popisu
			RVLQLIST_HIST_ENTRY *pHistEntryT;
			RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST_HIST_ENTRY, pHistEntryT);
			pHistEntryT->adr = adr;
			pHistEntryT->value = pElement->nPts;
			textonHist[adr] = pHistEntryT;
			RVLQLIST_ADD_ENTRY(textonHistList, pHistEntryT);
		}
		noTex++;
		pElement = (RVLTEXTON*)pElement->pNext;
	}
	//Normiranje
	for (int i = 0; i < noBins; i++)
	{
		if (textonHist[i] && (noPix > 0))
			((RVLQLIST_HIST_ENTRY*)textonHist[i])->value = ((RVLQLIST_HIST_ENTRY*)textonHist[i])->value / noPix;
	}
	//Sortiranje
	this->m_TextonHist = InsertSortQLISTWithCopy<RVLQLIST_HIST_ENTRY>(textonHistList, RVLQLIST_DESCENDING, this->m_pClass->m_pMem0, TRUE);
	if (noTex > 0)
		return (float)noPix / (float) noTex;
	else
		return -1.0;
}

//Normed transform RGB color space
void CRVL3DMeshObject::RVLCalculateRGBNTHist(IplImage* rgbImg, float* histBase, bool soft)
{
	int noBins = (int)histBase[0] * histBase[1] * histBase[2];
	float normOffset = 3.0;	//We are offseting the centar by 3 * sigma
	float ratioR = (2 * normOffset) / histBase[0]; //how many levels are per bin
	float ratioG = (2 * normOffset) / histBase[1]; //how many levels are per bin
	float ratioB = (2 * normOffset) / histBase[2]; //how many levels are per bin
	CRVL2DRegion2 *pTriangle;
	RVL3DPOINT2 **ppPt, **pPtArrayEnd;
	RVL3DPOINT2 *pPt;
	int iPix;
	int label;
	int r = 0, g = 0, b = 0, rgb_x = 0, rgb_y = 0;
	float rr = 0.0, gg = 0.0, bb = 0.0;
	int w = rgbImg->width;
	int h = rgbImg->height;
	int adr;

	int updateBins[8][3];	//Needed for soft histograms, coordinates for updated bins
	memset(updateBins, 0, 8 * 3 * sizeof(int));
	int bins2Update = 0;	//Needed for soft histograms, number of bins needed to update
	float binCentar[3];	//Needed for soft histograms, closest bin centar. Float because the values are float to begin with
	int bin[3];	//Needed for soft histograms, central bin coordinates
	int valuePos[3];	//current value above, below or on bin centar, per dimension(R,G,B)
	float binWeightCoeff[8][3];	//Needed for soft histograms, calculated coefficients for faster analysis
	float weightTypes[2][3];	//Needed for soft histograms, there can be only two weights, offset by 0 or +/- 1
	memset(binWeightCoeff, 0, 8 * 3 * sizeof(float));

	void** histRGB;// = new void*[noBins];
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, void*, noBins, histRGB);
	RVLQLIST *histRGBList;// = new RVLQLIST[1];
	RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST, histRGBList);

	void** objHistRGB;// = new void*[noBins];
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, void*, noBins, objHistRGB);
	for(int i = 0; i < noBins; i++)
	{
		if (objHistRGB[i])
			objHistRGB[i] = NULL;
	}
	RVLQLIST *objHistRGBList;// = new RVLQLIST[1];
	RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST, objHistRGBList);
	RVLQLIST_INIT(objHistRGBList);

	uchar *ptr;
	RVLQLIST_PTR_ENTRY *pElement;
	//Prvo obilazimo za racunanje standardne devijacije i srednje vrijednosti
	//varijable potrebne za on-line raèunanje varijance
	float mean[3] = {0.0, 0.0, 0.0};
	double M2[3] = {0.0, 0.0, 0.0};
	int noPts = 0;
	float delta = 0.0;
	pElement = (RVLQLIST_PTR_ENTRY*)this->m_FaceList->pFirst;
	while(pElement)
	{
		pTriangle = (CRVL2DRegion2 *)(pElement->Ptr);

		if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		pPtArrayEnd = pTriangle->m_pPoint3DArray + pTriangle->m_n3DPts;

		for(ppPt = pTriangle->m_pPoint3DArray; ppPt < pPtArrayEnd; ppPt++)
		{
			pPt = *ppPt;

			iPix = pPt->iPix;
			//ako je maska postavljena i nije dozvoljen idemo dalje
			if (pTriangle->m_uPixMask && (pTriangle->m_uPixMask[iPix] == 1))
				continue;
			//Povecavamo broj koristenih piksela s bojom
			//this->m_noUsedColorPts++;
			//Provjeriti
			rgb_y = pPt->iPixRGB / w;
			rgb_x = pPt->iPixRGB - rgb_y * w;
			if (rgb_y != 0)
				rgb_x --;
			ptr = (uchar*)(rgbImg->imageData + rgb_y * rgbImg->widthStep);
			if (rgbImg->channelSeq[0] == 'B')
			{
				b = ptr[rgb_x * 3];
				g = ptr[rgb_x * 3 + 1];
				r = ptr[rgb_x * 3 + 2];
			}
			else
			{
				r = ptr[rgb_x * 3];
				g = ptr[rgb_x * 3 + 1];
				b = ptr[rgb_x * 3 + 2];
			}
			noPts++;
			//R
			delta = r - mean[0];
			mean[0] += delta/(float)noPts;
			M2[0] += delta*(r - mean[0]);
			//G
			delta = g - mean[1];
			mean[1] += delta/(float)noPts;
			M2[1] += delta*(g - mean[1]);
			//B
			delta = b - mean[2];
			mean[2] += delta/(float)noPts;
			M2[2] += delta*(b - mean[2]);			
		}
		pElement = (RVLQLIST_PTR_ENTRY*)pElement->pNext;
	}
	float stDev[3] = {0.0, 0.0, 0.0};
	stDev[0] = sqrt(M2[0]/(float)noPts);
	stDev[1] = sqrt(M2[1]/(float)noPts);
	stDev[2] = sqrt(M2[2]/(float)noPts);

	//Proracunavamo histogram
	pElement = (RVLQLIST_PTR_ENTRY*)this->m_FaceList->pFirst;
	while(pElement)
	{
		pTriangle = (CRVL2DRegion2 *)(pElement->Ptr);

		if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		label = pTriangle->m_Label;

		pPtArrayEnd = pTriangle->m_pPoint3DArray + pTriangle->m_n3DPts;

		for(int i = 0; i < noBins; i++)
		{
			if (histRGB[i])
				histRGB[i] = NULL;
		}
		RVLQLIST_INIT(histRGBList);	//resetiramo listu 

		//pTriangle->m_histRGB;// = new RVLQLIST[1];
		RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST, pTriangle->m_histRGB);
		RVLQLIST_INIT(pTriangle->m_histRGB);
		pTriangle->m_histRGB_base[0] = histBase[0];
		pTriangle->m_histRGB_base[1] = histBase[1];
		pTriangle->m_histRGB_base[2] = histBase[2];
		if ((rgbImg->channelSeq[0] == 'B') || (rgbImg->channelSeq[0] == 'R'))
			pTriangle->m_ColorSystem = RVL_MESH_COLOR_NORM_TRANS_3D;

		for(ppPt = pTriangle->m_pPoint3DArray; ppPt < pPtArrayEnd; ppPt++)
		{
			pPt = *ppPt;

			iPix = pPt->iPix;
			//ako je maska postavljena i nije dozvoljen idemo dalje
			if (pTriangle->m_uPixMask && (pTriangle->m_uPixMask[iPix] == 1))
				continue;
			//Povecavamo broj koristenih piksela s bojom
			this->m_noUsedColorPts++;
			//Provjeriti
			rgb_y = pPt->iPixRGB / w;
			rgb_x = pPt->iPixRGB - rgb_y * w;
			if (rgb_y != 0)
				rgb_x --;
			ptr = (uchar*)(rgbImg->imageData + rgb_y * rgbImg->widthStep);
			if (rgbImg->channelSeq[0] == 'B')
			{
				b = ptr[rgb_x * 3];
				g = ptr[rgb_x * 3 + 1];
				r = ptr[rgb_x * 3 + 2];
			}
			else
			{
				r = ptr[rgb_x * 3];
				g = ptr[rgb_x * 3 + 1];
				b = ptr[rgb_x * 3 + 2];
			}

			rr = (float)(r - mean[0])/ stDev[0] + 3.0;	//prebacujemo iz intervala -3/+3 na 0/+6
			if (rr < 0.0)		//Ogranicavamo na samo +/- 3sigma
				rr = 0.0;
			else if (rr > 5.99)
				rr = 5.99;
			gg = (float)(g - mean[1])/ stDev[1] + 3.0;
			if (gg < 0.0)
				gg = 0.0;
			else if (gg > 5.99)
				gg = 5.99;
			bb = (float)(b - mean[2])/ stDev[2] + 3.0;
			if (bb < 0.0)
				bb = 0.0;
			else if (bb > 5.99)
				bb = 5.99;

			if(!soft)
			{
				adr = (int)(floor(rr / ratioR) * histBase[1] * histBase[2] + floor(gg / ratioG) * histBase[2] + floor(bb / ratioB));
				if (histRGB[adr])
				{
					//azuriramo lokalni trikutic
					((RVLQLIST_HIST_ENTRY*)histRGB[adr])->value++;
					//azuriramo objekt
					((RVLQLIST_HIST_ENTRY*)objHistRGB[adr])->value++;
				}
				else
				{
					//stvaramo novi element koji ce biti i u arrayu i u povezanom popisu
					RVLQLIST_HIST_ENTRY *pHistEntryT;// = new RVLQLIST_HIST_ENTRY_SHORT[1];
					RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST_HIST_ENTRY, pHistEntryT);
					pHistEntryT->adr = adr;
					pHistEntryT->value = 1;
					histRGB[adr] = pHistEntryT;
					RVLQLIST_ADD_ENTRY(histRGBList, pHistEntryT);
					
					if (!objHistRGB[adr])
					{
						RVLQLIST_HIST_ENTRY *pHistEntryO;// = new RVLQLIST_HIST_ENTRY_SHORT[1];
						RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST_HIST_ENTRY, pHistEntryO);
						pHistEntryO->adr = adr;
						pHistEntryO->value = 1;
						objHistRGB[adr] = pHistEntryO;
						RVLQLIST_ADD_ENTRY(objHistRGBList, pHistEntryO);
					}
					else
						((RVLQLIST_HIST_ENTRY*)objHistRGB[adr])->value++;	//azuriramo objekt
				}
			}
			else
			{
				//closest bin centar
				bin[0] = (int)floor(rr / ratioR);
				bin[1] = (int)floor(gg / ratioG);
				bin[2] = (int)floor(bb / ratioB);
				binCentar[0] = (float)bin[0] * ratioR + ratioR / 2.0;
				binCentar[1] = (float)bin[1] * ratioG + ratioG / 2.0;
				binCentar[2] = (float)bin[2] * ratioB + ratioB / 2.0;
				
				//is current value above bin centar per dimension
				if (rr > binCentar[0])
					valuePos[0] = 1;
				else if (rr == binCentar[0])
					valuePos[0] = 0;
				else if (rr < binCentar[0])
					valuePos[0] = -1;

				if (gg > binCentar[1])
					valuePos[1] = 1;
				else if (gg == binCentar[1])
					valuePos[1] = 0;
				else if (gg < binCentar[1])
					valuePos[1] = -1;

				if (bb > binCentar[2])
					valuePos[2] = 1;
				else if (bb == binCentar[2])
					valuePos[2] = 0;
				else if (bb < binCentar[2])
					valuePos[2] = -1;

				//Finding which bin to update
				updateBins[0][0] = bin[0];
				updateBins[0][1] = bin[1];
				updateBins[0][2] = bin[2];
				//Calculating weight types
				//'0' offset
				weightTypes[0][0] = (float)(1.0 - abs((float)(rr - ((float)updateBins[0][0] * ratioR + ratioR / 2.0))) / ratioR);
				weightTypes[0][1] = (float)(1.0 - abs((float)(gg - ((float)updateBins[0][1] * ratioG + ratioG / 2.0))) / ratioG);
				weightTypes[0][2] = (float)(1.0 - abs((float)(bb - ((float)updateBins[0][2] * ratioB + ratioB / 2.0))) / ratioB);
				//'+/- 1' offset
				weightTypes[1][0] = (float)(1.0 - abs((float)(rr - ((float)(bin[0] + valuePos[0]) * ratioR + ratioR / 2.0))) / ratioR);
				weightTypes[1][1] = (float)(1.0 - abs((float)(gg - ((float)(bin[1] + valuePos[1]) * ratioG + ratioG / 2.0))) / ratioG);
				weightTypes[1][2] = (float)(1.0 - abs((float)(bb - ((float)(bin[2] + valuePos[2]) * ratioB + ratioB / 2.0))) / ratioB);
				
				/*binWeightCoeff[0][0] = (float)(1.0 - abs((float)(r - (updateBins[0][0] * ratio + ratio / 2.0))) / ratio);
				binWeightCoeff[0][1] = (float)(1.0 - abs((float)(g - (updateBins[0][1] * ratio + ratio / 2.0))) / ratio);
				binWeightCoeff[0][2] = (float)(1.0 - abs((float)(b - (updateBins[0][2] * ratio + ratio / 2.0))) / ratio);*/
				binWeightCoeff[0][0] = weightTypes[0][0];
				binWeightCoeff[0][1] = weightTypes[0][1];
				binWeightCoeff[0][2] = weightTypes[0][2];
				bins2Update = 1;
				if (!((valuePos[0] == 0) && (valuePos[1] == 0) && (valuePos[2] == 0)))	//current color is in bin centar
				{
					//first coordinate
					if (valuePos[0] != 0)
					{
						updateBins[bins2Update][0] = bin[0] + valuePos[0];
						if ((updateBins[bins2Update][0] >= 0) && (updateBins[bins2Update][0] < histBase[0]))
						{
							updateBins[bins2Update][1] = bin[1];
							updateBins[bins2Update][2] = bin[2];
							/*binWeightCoeff[bins2Update][0] = (float)(1.0 - abs((float)(r - (updateBins[bins2Update][0] * ratio + ratio / 2.0))) / ratio);
							binWeightCoeff[bins2Update][1] = (float)(1.0 - abs((float)(g - (updateBins[bins2Update][1] * ratio + ratio / 2.0))) / ratio);
							binWeightCoeff[bins2Update][2] = (float)(1.0 - abs((float)(b - (updateBins[bins2Update][2] * ratio + ratio / 2.0))) / ratio);*/
							binWeightCoeff[bins2Update][0] = weightTypes[abs(bin[0] - updateBins[bins2Update][0])][0];
							binWeightCoeff[bins2Update][1] = weightTypes[abs(bin[1] - updateBins[bins2Update][1])][1];
							binWeightCoeff[bins2Update][2] = weightTypes[abs(bin[2] - updateBins[bins2Update][2])][2];
							bins2Update++;

							//first & second coordinate
							updateBins[bins2Update][1] = bin[1] + valuePos[1];
							if ((updateBins[bins2Update][1] >= 0) && (updateBins[bins2Update][1] < histBase[1]))
							{
								updateBins[bins2Update][0] = bin[0] + valuePos[0];
								updateBins[bins2Update][2] = bin[2];
								/*binWeightCoeff[bins2Update][0] = (float)(1.0 - abs((float)(r - (updateBins[bins2Update][0] * ratio + ratio / 2.0))) / ratio);
								binWeightCoeff[bins2Update][1] = (float)(1.0 - abs((float)(g - (updateBins[bins2Update][1] * ratio + ratio / 2.0))) / ratio);
								binWeightCoeff[bins2Update][2] = (float)(1.0 - abs((float)(b - (updateBins[bins2Update][2] * ratio + ratio / 2.0))) / ratio);*/
								binWeightCoeff[bins2Update][0] = weightTypes[abs(bin[0] - updateBins[bins2Update][0])][0];
								binWeightCoeff[bins2Update][1] = weightTypes[abs(bin[1] - updateBins[bins2Update][1])][1];
								binWeightCoeff[bins2Update][2] = weightTypes[abs(bin[2] - updateBins[bins2Update][2])][2];
								bins2Update++;
							}

							//first & third coordinate
							updateBins[bins2Update][2] = bin[2] + valuePos[2];
							if ((updateBins[bins2Update][2] >= 0)  && (updateBins[bins2Update][2] < histBase[2]))
							{
								updateBins[bins2Update][0] = bin[0] + valuePos[0];
								updateBins[bins2Update][1] = bin[1];
								/*binWeightCoeff[bins2Update][0] = (float)(1.0 - abs((float)(r - (updateBins[bins2Update][0] * ratio + ratio / 2.0))) / ratio);
								binWeightCoeff[bins2Update][1] = (float)(1.0 - abs((float)(g - (updateBins[bins2Update][1] * ratio + ratio / 2.0))) / ratio);
								binWeightCoeff[bins2Update][2] = (float)(1.0 - abs((float)(b - (updateBins[bins2Update][2] * ratio + ratio / 2.0))) / ratio);*/
								binWeightCoeff[bins2Update][0] = weightTypes[abs(bin[0] - updateBins[bins2Update][0])][0];
								binWeightCoeff[bins2Update][1] = weightTypes[abs(bin[1] - updateBins[bins2Update][1])][1];
								binWeightCoeff[bins2Update][2] = weightTypes[abs(bin[2] - updateBins[bins2Update][2])][2];
								bins2Update++;
							}
						}
					}

					//second coordinate
					if (valuePos[1] != 0)
					{
						updateBins[bins2Update][1] = bin[1] + valuePos[1];
						if ((updateBins[bins2Update][1] >= 0)  && (updateBins[bins2Update][1] < histBase[1]))
						{
							updateBins[bins2Update][0] = bin[0];
							updateBins[bins2Update][2] = bin[2];
							/*binWeightCoeff[bins2Update][0] = (float)(1.0 - abs((float)(r - (updateBins[bins2Update][0] * ratio + ratio / 2.0))) / ratio);
							binWeightCoeff[bins2Update][1] = (float)(1.0 - abs((float)(g - (updateBins[bins2Update][1] * ratio + ratio / 2.0))) / ratio);
							binWeightCoeff[bins2Update][2] = (float)(1.0 - abs((float)(b - (updateBins[bins2Update][2] * ratio + ratio / 2.0))) / ratio);*/
							binWeightCoeff[bins2Update][0] = weightTypes[abs(bin[0] - updateBins[bins2Update][0])][0];
							binWeightCoeff[bins2Update][1] = weightTypes[abs(bin[1] - updateBins[bins2Update][1])][1];
							binWeightCoeff[bins2Update][2] = weightTypes[abs(bin[2] - updateBins[bins2Update][2])][2];
							bins2Update++;

							//second & third coordinate
							updateBins[bins2Update][2] = bin[2] + valuePos[2];
							if ((updateBins[bins2Update][2] >= 0) && (updateBins[bins2Update][2] < histBase[2]))
							{
								updateBins[bins2Update][0] = bin[0];
								updateBins[bins2Update][1] = bin[1] + valuePos[1];
								/*binWeightCoeff[bins2Update][0] = (float)(1.0 - abs((float)(r - (updateBins[bins2Update][0] * ratio + ratio / 2.0))) / ratio);
								binWeightCoeff[bins2Update][1] = (float)(1.0 - abs((float)(g - (updateBins[bins2Update][1] * ratio + ratio / 2.0))) / ratio);
								binWeightCoeff[bins2Update][2] = (float)(1.0 - abs((float)(b - (updateBins[bins2Update][2] * ratio + ratio / 2.0))) / ratio);*/
								binWeightCoeff[bins2Update][0] = weightTypes[abs(bin[0] - updateBins[bins2Update][0])][0];
								binWeightCoeff[bins2Update][1] = weightTypes[abs(bin[1] - updateBins[bins2Update][1])][1];
								binWeightCoeff[bins2Update][2] = weightTypes[abs(bin[2] - updateBins[bins2Update][2])][2];
								bins2Update++;
							}
						}
					}
					//third coordinate
					if (valuePos[2] != 0)
					{
						updateBins[bins2Update][2] = bin[2] + valuePos[2];
						if ((updateBins[bins2Update][2] >= 0) && (updateBins[bins2Update][2] < histBase[2]))
						{
							updateBins[bins2Update][0] = bin[0];
							updateBins[bins2Update][1] = bin[1];
							/*binWeightCoeff[bins2Update][0] = (float)(1.0 - abs((float)(r - (updateBins[bins2Update][0] * ratio + ratio / 2.0))) / ratio);
							binWeightCoeff[bins2Update][1] = (float)(1.0 - abs((float)(g - (updateBins[bins2Update][1] * ratio + ratio / 2.0))) / ratio);
							binWeightCoeff[bins2Update][2] = (float)(1.0 - abs((float)(b - (updateBins[bins2Update][2] * ratio + ratio / 2.0))) / ratio);*/
							binWeightCoeff[bins2Update][0] = weightTypes[abs(bin[0] - updateBins[bins2Update][0])][0];
							binWeightCoeff[bins2Update][1] = weightTypes[abs(bin[1] - updateBins[bins2Update][1])][1];
							binWeightCoeff[bins2Update][2] = weightTypes[abs(bin[2] - updateBins[bins2Update][2])][2];
							bins2Update++;
						}
					}

					//All three coordinates
					updateBins[bins2Update][0] = bin[0] + valuePos[0];
					updateBins[bins2Update][1] = bin[1] + valuePos[1];
					updateBins[bins2Update][2] = bin[2] + valuePos[2];
					if ((updateBins[bins2Update][0] >= 0) && (updateBins[bins2Update][0] < histBase[0]) && (updateBins[bins2Update][1] >= 0) && (updateBins[bins2Update][1] < histBase[1]) && (updateBins[bins2Update][2] >= 0) && (updateBins[bins2Update][2] < histBase[2]))
					{
						/*binWeightCoeff[bins2Update][0] = (float)(1.0 - abs((float)(r - (updateBins[bins2Update][0] * ratio + ratio / 2.0))) / ratio);
						binWeightCoeff[bins2Update][1] = (float)(1.0 - abs((float)(g - (updateBins[bins2Update][1] * ratio + ratio / 2.0))) / ratio);
						binWeightCoeff[bins2Update][2] = (float)(1.0 - abs((float)(b - (updateBins[bins2Update][2] * ratio + ratio / 2.0))) / ratio);*/
						binWeightCoeff[bins2Update][0] = weightTypes[abs(bin[0] - updateBins[bins2Update][0])][0];
						binWeightCoeff[bins2Update][1] = weightTypes[abs(bin[1] - updateBins[bins2Update][1])][1];
						binWeightCoeff[bins2Update][2] = weightTypes[abs(bin[2] - updateBins[bins2Update][2])][2];
						bins2Update++;
					}
				}

				//Updating all taged bins
				for (int i = 0; i < bins2Update; i++)
				{
					adr = (int)(updateBins[i][0] * histBase[1] * histBase[2] + updateBins[i][1] * histBase[2] + updateBins[i][2]); // i = r/(256/16) * 16^2 + g/(256/16) * 16 + b/(256/16)

					if (histRGB[adr])
					{
						//azuriramo lokalni trikutic
						((RVLQLIST_HIST_ENTRY*)histRGB[adr])->value += binWeightCoeff[i][0] * binWeightCoeff[i][1] * binWeightCoeff[i][2];
						//azuriramo objekt
						((RVLQLIST_HIST_ENTRY*)objHistRGB[adr])->value += binWeightCoeff[i][0] * binWeightCoeff[i][1] * binWeightCoeff[i][2];
					}
					else
					{
						//stvaramo novi element koji ce biti i u arrayu i u povezanom popisu
						RVLQLIST_HIST_ENTRY *pHistEntryT;// = new RVLQLIST_HIST_ENTRY_SHORT[1];
						RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST_HIST_ENTRY, pHistEntryT);
						pHistEntryT->adr = adr;
						pHistEntryT->value = binWeightCoeff[i][0] * binWeightCoeff[i][1] * binWeightCoeff[i][2];
						histRGB[adr] = pHistEntryT;
						RVLQLIST_ADD_ENTRY(histRGBList, pHistEntryT);
						
						if (!objHistRGB[adr])
						{
							RVLQLIST_HIST_ENTRY *pHistEntryO;// = new RVLQLIST_HIST_ENTRY_SHORT[1];
							RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST_HIST_ENTRY, pHistEntryO);
							pHistEntryO->adr = adr;
							pHistEntryO->value = binWeightCoeff[i][0] * binWeightCoeff[i][1] * binWeightCoeff[i][2];
							objHistRGB[adr] = pHistEntryO;
							RVLQLIST_ADD_ENTRY(objHistRGBList, pHistEntryO);
						}
						else
							((RVLQLIST_HIST_ENTRY*)objHistRGB[adr])->value += binWeightCoeff[i][0] * binWeightCoeff[i][1] * binWeightCoeff[i][2];	//azuriramo objekt
					}

				}

			}
			
		}
		//Stvaramo i sortiramo RGB histogram za trokut
		pTriangle->m_histRGB = InsertSortQLISTWithCopy<RVLQLIST_HIST_ENTRY>(histRGBList, RVLQLIST_DESCENDING, this->m_pClass->m_pMem0, TRUE);
		pElement = (RVLQLIST_PTR_ENTRY*)pElement->pNext;
	}
	//Stvaramo i sortiramo RGB histogram za cijeli objekt
	this->m_histRGB = InsertSortQLISTWithCopy<RVLQLIST_HIST_ENTRY>(objHistRGBList, RVLQLIST_DESCENDING, this->m_pClass->m_pMem0, TRUE);
	this->m_histRGB_base[0] = histBase[0];
	this->m_histRGB_base[1] = histBase[1];
	this->m_histRGB_base[2] = histBase[2];
	if ((rgbImg->channelSeq[0] == 'B') || (rgbImg->channelSeq[0] == 'R'))
		this->m_ColorSystem = RVL_MESH_COLOR_NORM_TRANS_3D;

	//Azuriramo histogram od parenta ako postoji(NETESTIRANO!!!! STO AKO IMAJU RAZLICITE BAZE?!?!?)
	if (((CRVL3DMeshObject*)this->parentMeshObject)->m_histRGB)
	{
		RVLQLIST *pParentHistList = ((CRVL3DMeshObject*)this->parentMeshObject)->m_histRGB;
		if (pParentHistList->pFirst)
		{
			RVLQLIST_HIST_ENTRY *pParentHistEntry = (RVLQLIST_HIST_ENTRY *)pParentHistList->pFirst;
			RVLQLIST_HIST_ENTRY *pObjHistEntry = (RVLQLIST_HIST_ENTRY *)this->m_histRGB->pFirst;
			bool insert = FALSE;
			while(pObjHistEntry)
			{
				while (pParentHistEntry)
				{
					if (pObjHistEntry->adr == pParentHistEntry->adr)
						pParentHistEntry->value += pObjHistEntry->value;
					pParentHistEntry = (RVLQLIST_HIST_ENTRY *)pParentHistEntry->pNext;
					if (!pParentHistEntry)
					{
						RVLQLIST_ADD_ENTRY(pParentHistList, pObjHistEntry);
						insert = TRUE;
						break;
					}
				}
				pObjHistEntry = (RVLQLIST_HIST_ENTRY *)pObjHistEntry->pNext;
			}
			if (insert)
				((CRVL3DMeshObject*)this->parentMeshObject)->m_histRGB = InsertSortQLISTWithCopy<RVLQLIST_HIST_ENTRY>(pParentHistList, RVLQLIST_DESCENDING, this->m_pClass->m_pMem0, TRUE);

		}
		else
		{
			//Kopiraj podatke od objekta
			RVLQLIST_HIST_ENTRY *pObjHistEntry = (RVLQLIST_HIST_ENTRY *)this->m_histRGB->pFirst;
			while(pObjHistEntry)
			{
				RVLQLIST_HIST_ENTRY *pParentHistNewEntry;// = new RVLQLIST_HIST_ENTRY_SHORT[1];
				RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem0, RVLQLIST_HIST_ENTRY, pParentHistNewEntry);
				pParentHistNewEntry->adr = pObjHistEntry->adr;
				pParentHistNewEntry->value = pObjHistEntry->value;
				RVLQLIST_ADD_ENTRY(pParentHistList, pParentHistNewEntry);

				pObjHistEntry = (RVLQLIST_HIST_ENTRY *)pObjHistEntry->pNext;
			}
		}
	}		
}

void CRVL3DMeshObject::RVLCalculateRGBOppHist(IplImage* rgbImg, float* histBase, bool soft)
{
	int noBins = (int)histBase[0] * histBase[1];
	float root2 = sqrt(2.0);
	float root6 = sqrt(6.0);
	float halfO1 = 256.0 / root2;
	float halfO2 = 512.0 / root6;
	float ratioO1 = (2 * halfO1) / histBase[0]; //how many levels are per bin
	float ratioO2 = (2 * halfO2) / histBase[1]; //how many levels are per bin
	CRVL2DRegion2 *pTriangle;
	RVL3DPOINT2 **ppPt, **pPtArrayEnd;
	RVL3DPOINT2 *pPt;
	int iPix;
	int label;
	int r = 0, g = 0, b = 0, rgb_x = 0, rgb_y = 0;
	float oo1 = 0.0, oo2 = 0.0;
	int w = rgbImg->width;
	int h = rgbImg->height;
	int adr;

	int updateBins[4][2];	//Needed for soft histograms, coordinates for updated bins
	memset(updateBins, 0, 4 * 2 * sizeof(int));
	int bins2Update = 0;	//Needed for soft histograms, number of bins needed to update
	float binCentar[2];	//Needed for soft histograms, closest bin centar
	int bin[2];	//Needed for soft histograms, central bin coordinates
	int valuePos[2];	//current value above, below or on bin centar, per dimension(r,g)
	float binWeightCoeff[4][2];	//Needed for soft histograms, calculated coefficients for faster analysis
	float weightTypes[2][2];	//Needed for soft histograms, there can be only two weights, offset by 0 or +/- 1
	memset(binWeightCoeff, 0, 4 * 2 * sizeof(float));

	void** histRGB;// = new void*[noBins];
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, void*, noBins, histRGB);
	RVLQLIST *histRGBList;// = new RVLQLIST[1];
	RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST, histRGBList);

	void** objHistRGB;// = new void*[noBins];
	RVLMEM_ALLOC_STRUCT_ARRAY(this->m_pClass->m_pMem2, void*, noBins, objHistRGB);
	for(int i = 0; i < noBins; i++)
	{
		if (objHistRGB[i])
			objHistRGB[i] = NULL;
	}
	RVLQLIST *objHistRGBList;// = new RVLQLIST[1];
	RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST, objHistRGBList);
	RVLQLIST_INIT(objHistRGBList);

	uchar *ptr;
	RVLQLIST_PTR_ENTRY *pElement;
	pElement = (RVLQLIST_PTR_ENTRY*)this->m_FaceList->pFirst;

	while(pElement)
	{
		pTriangle = (CRVL2DRegion2 *)(pElement->Ptr);

		if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)
			continue;

		label = pTriangle->m_Label;

		pPtArrayEnd = pTriangle->m_pPoint3DArray + pTriangle->m_n3DPts;

		for(int i = 0; i < noBins; i++)
		{
			if (histRGB[i])
				histRGB[i] = NULL;
		}
		RVLQLIST_INIT(histRGBList);	//resetiramo listu 

		//pTriangle->m_histRGB;// = new RVLQLIST[1];
		RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST, pTriangle->m_histRGB);
		RVLQLIST_INIT(pTriangle->m_histRGB);
		pTriangle->m_histRGB_base[0] = histBase[0];
		pTriangle->m_histRGB_base[1] = histBase[1];
		pTriangle->m_histRGB_base[2] = histBase[2];
		pTriangle->m_ColorSystem = RVL_MESH_COLOR_OPPONENT_2D;

		for(ppPt = pTriangle->m_pPoint3DArray; ppPt < pPtArrayEnd; ppPt++)
		{
			pPt = *ppPt;

			iPix = pPt->iPix;
			//ako je maska postavljena i nije dozvoljen idemo dalje
			if (pTriangle->m_uPixMask && (pTriangle->m_uPixMask[iPix] == 1))
				continue;
			//Povecavamo broj koristenih piksela s bojom
			this->m_noUsedColorPts++;
			//Provjeriti
			rgb_y = pPt->iPixRGB / w;
			rgb_x = pPt->iPixRGB - rgb_y * w;
			if (rgb_y != 0)
				rgb_x --;
			ptr = (uchar*)(rgbImg->imageData + rgb_y * rgbImg->widthStep);
			if (rgbImg->channelSeq[0] == 'B')
			{
				b = ptr[rgb_x * 3];
				g = ptr[rgb_x * 3 + 1];
				r = ptr[rgb_x * 3 + 2];
			}
			else
			{
				r = ptr[rgb_x * 3];
				g = ptr[rgb_x * 3 + 1];
				b = ptr[rgb_x * 3 + 2];
			}

			oo1 = (float)(r - g) / root2 + halfO1;
			oo2 = (float)(r + g - 2 * b) / root6 + halfO2;

			if(!soft)
			{
				adr = (int)(floor(oo1 / ratioO1) * histBase[1] + floor(oo2 / ratioO2)); // 
				if (histRGB[adr])
				{
					//azuriramo lokalni trikutic
					((RVLQLIST_HIST_ENTRY*)histRGB[adr])->value++;
					//azuriramo objekt
					((RVLQLIST_HIST_ENTRY*)objHistRGB[adr])->value++;
				}
				else
				{
					//stvaramo novi element koji ce biti i u arrayu i u povezanom popisu
					RVLQLIST_HIST_ENTRY *pHistEntryT;// = new RVLQLIST_HIST_ENTRY_SHORT[1];
					RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST_HIST_ENTRY, pHistEntryT);
					pHistEntryT->adr = adr;
					pHistEntryT->value = 1;
					histRGB[adr] = pHistEntryT;
					RVLQLIST_ADD_ENTRY(histRGBList, pHistEntryT);
					
					if (!objHistRGB[adr])
					{
						RVLQLIST_HIST_ENTRY *pHistEntryO;// = new RVLQLIST_HIST_ENTRY_SHORT[1];
						RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST_HIST_ENTRY, pHistEntryO);
						pHistEntryO->adr = adr;
						pHistEntryO->value = 1;
						objHistRGB[adr] = pHistEntryO;
						RVLQLIST_ADD_ENTRY(objHistRGBList, pHistEntryO);
					}
					else
						((RVLQLIST_HIST_ENTRY*)objHistRGB[adr])->value++;	//azuriramo objekt
				}
			}
			else
			{
				//closest bin centar
				bin[0] = (int)floor(oo1 / ratioO1);
				bin[1] = (int)floor(oo2 / ratioO2);
				binCentar[0] = (float)(bin[0] * ratioO1 + ratioO1 / 2);
				binCentar[1] = (float)(bin[1] * ratioO2 + ratioO2 / 2);
				
				//is current value above bin centar per dimension
				if (oo1 > binCentar[0])
					valuePos[0] = 1;
				else if (oo1 == binCentar[0])
					valuePos[0] = 0;
				else if (oo1 < binCentar[0])
					valuePos[0] = -1;

				if (oo2 > binCentar[1])
					valuePos[1] = 1;
				else if (oo2 == binCentar[1])
					valuePos[1] = 0;
				else if (oo2 < binCentar[1])
					valuePos[1] = -1;

				//Finding which bin to update
				updateBins[0][0] = bin[0];
				updateBins[0][1] = bin[1];
				//Calculating weight types
				//'0' offset
				weightTypes[0][0] = (float)(1.0 - abs((float)(oo1 - (updateBins[0][0] * ratioO1 + ratioO1 / 2.0))) / ratioO1);
				weightTypes[0][1] = (float)(1.0 - abs((float)(oo2 - (updateBins[0][1] * ratioO2 + ratioO2 / 2.0))) / ratioO2);
				//'+/- 1' offset
				weightTypes[1][0] = (float)(1.0 - abs((float)(oo1 - ((bin[0] + valuePos[0]) * ratioO1 + ratioO1 / 2.0))) / ratioO1);
				weightTypes[1][1] = (float)(1.0 - abs((float)(oo2 - ((bin[1] + valuePos[1]) * ratioO2 + ratioO2 / 2.0))) / ratioO2);
				
				binWeightCoeff[0][0] = weightTypes[0][0];
				binWeightCoeff[0][1] = weightTypes[0][1];
				bins2Update = 1;

				if (!((valuePos[0] == 0) && (valuePos[1] == 0)))	//current color is in bin centar
				{
					//first coordinate
					if (valuePos[0] != 0)
					{
						updateBins[bins2Update][0] = bin[0] + valuePos[0];
						if ((updateBins[bins2Update][0] >= 0) && (updateBins[bins2Update][0] < histBase[0]))
						{
							updateBins[bins2Update][1] = bin[1];
							
							binWeightCoeff[bins2Update][0] = weightTypes[abs(bin[0] - updateBins[bins2Update][0])][0];
							binWeightCoeff[bins2Update][1] = weightTypes[abs(bin[1] - updateBins[bins2Update][1])][1];
							bins2Update++;

							//first & second coordinate
							updateBins[bins2Update][1] = bin[1] + valuePos[1];
							if ((updateBins[bins2Update][1] >= 0) && (updateBins[bins2Update][1] < histBase[1]))
							{
								updateBins[bins2Update][0] = bin[0] + valuePos[0];
								binWeightCoeff[bins2Update][0] = weightTypes[abs(bin[0] - updateBins[bins2Update][0])][0];
								binWeightCoeff[bins2Update][1] = weightTypes[abs(bin[1] - updateBins[bins2Update][1])][1];
								bins2Update++;
							}
						}
					}

					//second coordinate
					if (valuePos[1] != 0)
					{
						updateBins[bins2Update][1] = bin[1] + valuePos[1];
						if ((updateBins[bins2Update][1] >= 0)  && (updateBins[bins2Update][1] < histBase[1]))
						{
							updateBins[bins2Update][0] = bin[0];
							binWeightCoeff[bins2Update][0] = weightTypes[abs(bin[0] - updateBins[bins2Update][0])][0];
							binWeightCoeff[bins2Update][1] = weightTypes[abs(bin[1] - updateBins[bins2Update][1])][1];
							bins2Update++;
						}
					}
				}

				//Updating all taged bins
				for (int i = 0; i < bins2Update; i++)
				{
					adr = (int)(updateBins[i][0] * histBase[1] + updateBins[i][1]);

					if (histRGB[adr])
					{
						//azuriramo lokalni trokutic
						((RVLQLIST_HIST_ENTRY*)histRGB[adr])->value += binWeightCoeff[i][0] * binWeightCoeff[i][1];
						//azuriramo objekt
						((RVLQLIST_HIST_ENTRY*)objHistRGB[adr])->value += binWeightCoeff[i][0] * binWeightCoeff[i][1];
					}
					else
					{
						//stvaramo novi element koji ce biti i u arrayu i u povezanom popisu
						RVLQLIST_HIST_ENTRY *pHistEntryT;// = new RVLQLIST_HIST_ENTRY_SHORT[1];
						RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST_HIST_ENTRY, pHistEntryT);
						pHistEntryT->adr = adr;
						pHistEntryT->value = binWeightCoeff[i][0] * binWeightCoeff[i][1];
						histRGB[adr] = pHistEntryT;
						RVLQLIST_ADD_ENTRY(histRGBList, pHistEntryT);
						
						if (!objHistRGB[adr])
						{
							RVLQLIST_HIST_ENTRY *pHistEntryO;// = new RVLQLIST_HIST_ENTRY_SHORT[1];
							RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem2, RVLQLIST_HIST_ENTRY, pHistEntryO);
							pHistEntryO->adr = adr;
							pHistEntryO->value = binWeightCoeff[i][0] * binWeightCoeff[i][1];
							objHistRGB[adr] = pHistEntryO;
							RVLQLIST_ADD_ENTRY(objHistRGBList, pHistEntryO);
						}
						else
							((RVLQLIST_HIST_ENTRY*)objHistRGB[adr])->value += binWeightCoeff[i][0] * binWeightCoeff[i][1];	//azuriramo objekt
					}

				}

			}
			
		}
		//Stvaramo i sortiramo RGB histogram za trokut
		pTriangle->m_histRGB = InsertSortQLISTWithCopy<RVLQLIST_HIST_ENTRY>(histRGBList, RVLQLIST_DESCENDING, this->m_pClass->m_pMem0, TRUE);
		pElement = (RVLQLIST_PTR_ENTRY*)pElement->pNext;
	}
	//Stvaramo i sortiramo RGB histogram za cijeli objekt
	this->m_histRGB = InsertSortQLISTWithCopy<RVLQLIST_HIST_ENTRY>(objHistRGBList, RVLQLIST_DESCENDING, this->m_pClass->m_pMem0, TRUE);
	this->m_histRGB_base[0] = histBase[0];
	this->m_histRGB_base[1] = histBase[1];
	this->m_histRGB_base[2] = histBase[2];
	this->m_ColorSystem = RVL_MESH_COLOR_OPPONENT_2D;

	//Azuriramo histogram od parenta ako postoji(NETESTIRANO!!!! STO AKO IMAJU RAZLICITE BAZE?!?!?)
	if (((CRVL3DMeshObject*)this->parentMeshObject)->m_histRGB)
	{
		RVLQLIST *pParentHistList = ((CRVL3DMeshObject*)this->parentMeshObject)->m_histRGB;
		if (pParentHistList->pFirst)
		{
			RVLQLIST_HIST_ENTRY *pParentHistEntry = (RVLQLIST_HIST_ENTRY *)pParentHistList->pFirst;
			RVLQLIST_HIST_ENTRY *pObjHistEntry = (RVLQLIST_HIST_ENTRY *)this->m_histRGB->pFirst;
			bool insert = FALSE;
			while(pObjHistEntry)
			{
				while (pParentHistEntry)
				{
					if (pObjHistEntry->adr == pParentHistEntry->adr)
						pParentHistEntry->value += pObjHistEntry->value;
					pParentHistEntry = (RVLQLIST_HIST_ENTRY *)pParentHistEntry->pNext;
					if (!pParentHistEntry)
					{
						RVLQLIST_ADD_ENTRY(pParentHistList, pObjHistEntry);
						insert = TRUE;
						break;
					}
				}
				pObjHistEntry = (RVLQLIST_HIST_ENTRY *)pObjHistEntry->pNext;
			}
			if (insert)
				((CRVL3DMeshObject*)this->parentMeshObject)->m_histRGB = InsertSortQLISTWithCopy<RVLQLIST_HIST_ENTRY>(pParentHistList, RVLQLIST_DESCENDING, this->m_pClass->m_pMem0, TRUE);

		}
		else
		{
			//Kopiraj podatke od objekta
			RVLQLIST_HIST_ENTRY *pObjHistEntry = (RVLQLIST_HIST_ENTRY *)this->m_histRGB->pFirst;
			while(pObjHistEntry)
			{
				RVLQLIST_HIST_ENTRY *pParentHistNewEntry;// = new RVLQLIST_HIST_ENTRY_SHORT[1];
				RVLMEM_ALLOC_STRUCT(this->m_pClass->m_pMem0, RVLQLIST_HIST_ENTRY, pParentHistNewEntry);
				pParentHistNewEntry->adr = pObjHistEntry->adr;
				pParentHistNewEntry->value = pObjHistEntry->value;
				RVLQLIST_ADD_ENTRY(pParentHistList, pParentHistNewEntry);

				pObjHistEntry = (RVLQLIST_HIST_ENTRY *)pObjHistEntry->pNext;
			}
		}
	}	
}

CRVL3DMeshObject* GenMeshObjects(CRVLMPtrChain *pTriangleList, 
								 IplImage*pImg, int nObjects, 
								 CRVLClass *pClass)

{

      CRVL3DMeshObject* objects;// = new CRVL3DMeshObject[nObjects];

      RVLMEM_ALLOC_STRUCT_ARRAY(pClass->m_pMem2, CRVL3DMeshObject, nObjects, objects);

      //initializing objects

      CRVL3DMeshObject* parent = new CRVL3DMeshObject();

      parent->m_pClass = pClass;

      parent->InitParent();

      RVLQLIST_PTR_ENTRY *pElement;

      for (int i = 0; i < nObjects; i++)

      {

            objects[i].m_pClass = pClass;

            objects[i].Init();

            objects[i].InitChild();

            objects[i].rootMeshObject = parent;

            objects[i].parentMeshObject = parent;

            RVLMEM_ALLOC_STRUCT(pClass->m_pMem2, RVLQLIST_PTR_ENTRY, pElement);

            pElement->Ptr = &objects[i];

            RVLQLIST_ADD_ENTRY(parent->m_ChildMeshObjects, pElement);

      }
	
      //Iterating through triangles

      RVL3DPOINT2 **ppPt, **pPtArrayEnd; //needed for setting iPixRGB

      CRVL2DRegion2 *pTriangle;

      pTriangleList->Start();

      while(pTriangleList->m_pNext)

      {

            pTriangle = (CRVL2DRegion2 *)(pTriangleList->GetNext());

            // pTriangle is a pointer to an instance of the class

            // CRVL2DRegion2 representing a mesh triangle. 

            // Now you can do whatever you want with the triangle.

            if(pTriangle->m_Flags & RVLOBJ2_FLAG_REJECTED)

                  continue;

            RVLQLIST_PTR_ENTRY *pEntry;

            RVLMEM_ALLOC_STRUCT(pClass->m_pMem, RVLQLIST_PTR_ENTRY, pEntry);

            pEntry->Ptr = pTriangle;

            RVLQLIST_ADD_ENTRY(objects[pTriangle->m_Label].m_FaceList, pEntry);

            objects[pTriangle->m_Label].m_noFaces++;

 

            //Running through all points (because of registration, depth and RGB pixels have same adress)

            pPtArrayEnd = pTriangle->m_pPoint3DArray + pTriangle->m_n3DPts;

            for(ppPt = pTriangle->m_pPoint3DArray; ppPt < pPtArrayEnd; ppPt++)

                  (*ppPt)->iPixRGB = (*ppPt)->iPix;                    

      }

 

      ////Generating Histograms

      //float histBase[] = {16.0, 16.0, 16.0}; //bins per dimension of color histogram

      //for (int i = 0; i < nObjects; i++)

      //{

      //      //objects[i].RVLCalculateHSVHist(pImg, histBase, false);

      //      objects[i].RVLCalculateRGBHist(pImg, histBase, false);

      //}

 

      //returning objects

      //return objects;

      return parent;

}
 

//void PruneTrianglesFromObjects(CRVL3DMeshObject* objects, 
//							   int nObjects)
//
//{
//      CRVL2DRegion2 *pTriangle;
//
//      RVLQLIST_PTR_ENTRY *pElement;
//	  float intersectVal = 0.0;
//	  float *intersectionHelperArray = new float[32*32];
//
//      //iterating through objects
//
//      for (int i = 0; i < nObjects; i++)
//
//      {
//		  float nMatchedPoints = 0.0;
//		  float nSaturatedPoints = 0.0;
//		  int hue, sat;
//		  int base = (int)(objects[i].m_histRGB_base[0]);
//			RVLQLIST_HIST_ENTRY *pHistEntry;
//			pHistEntry = (RVLQLIST_HIST_ENTRY*)(objects[i].m_histRGB->pFirst);
//			while(pHistEntry)
//			{
//				hue = pHistEntry->adr / base;
//				sat = pHistEntry->adr - hue * base;
//				if(sat >= 1)
//				{
//					nSaturatedPoints += pHistEntry->value;
//
//					//if(hue == 0 || hue == 6 || hue == 7)
//					//if(hue >= 1 && hue <= 4)
//					if(hue == 4 || hue == 5)
//						nMatchedPoints += pHistEntry->value;
//				}
//				pHistEntry = (RVLQLIST_HIST_ENTRY*)pHistEntry->pNext;
//			}
//
//			if(nMatchedPoints / nSaturatedPoints >= 0.8)
//			{
//				pElement = (RVLQLIST_PTR_ENTRY*)objects[i].m_FaceList->pFirst;   //first element
//
//				while(pElement)
//				{
//					  pTriangle = (CRVL2DRegion2 *)(pElement->Ptr);
//
//					  pTriangle->m_Flags |= RVLOBJ2_FLAG_MARKED;
//					  //pTriangle->m_Label = -1;
//
//					  pElement = (RVLQLIST_PTR_ENTRY*)pElement->pNext;
//				}
//			}
//
//      //      pElement = (RVLQLIST_PTR_ENTRY*)objects[i].m_FaceList->pFirst;   //first element
//
//      //      while(pElement)
//
//      //      {
//
//      //            pTriangle = (CRVL2DRegion2 *)(pElement->Ptr);
//
//				  //intersectVal = CRVL3DMeshObject::RVLIntersectHistograms(pTriangle->m_histRGB, objects[i].m_histRGB, pTriangle->m_n3DPts, objects[i].m_noUsedColorPts, objects[i].m_histRGB_base[0]*objects[i].m_histRGB_base[1], objects[i].m_pClass->m_pMem2, 2, intersectionHelperArray);
//				  ////intersectVal = CRVL3DMeshObject::RVLIntersectHistograms(objects[i].m_histRGB, pTriangle->m_histRGB, objects[i].m_noUsedColorPts, pTriangle->m_n3DPts, objects[i].m_histRGB_base[0]*objects[i].m_histRGB_base[1], objects[i].m_pClass->m_pMem2, 2, intersectionHelperArray);
//      //            //next element
//
//			   //   if(intersectVal < 0.15)
//				  //{
//					 // pTriangle->m_Flags |= RVLOBJ2_FLAG_REJECTED;
//					 // pTriangle->m_Label = -1;
//				  //}
//
//      //            pElement = (RVLQLIST_PTR_ENTRY*)pElement->pNext;
//
//					
//      //      }
//
//      }
//	  delete [] intersectionHelperArray;
//}

