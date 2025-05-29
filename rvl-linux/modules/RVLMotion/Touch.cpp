#include "RVLCore2.h"
#include "RVLVTK.h"
#include "Util.h"
#include "Space3DGrid.h"
#include "SE3Grid.h"
#include "Graph.h"
#include "Mesh.h"
#include "Visualizer.h"
#include "SceneSegFile.hpp"
#include "SurfelGraph.h"
#include "ObjectGraph.h"
#include "PlanarSurfelDetector.h"
#include "RVLRecognition.h"
#include "RVLRecognitionCommon.h"
#include "PSGMCommon.h"
#include "CTISet.h"
#include "VertexGraph.h"
#include "TG.h"
#include "TGSet.h"
#include "PSGM.h"
#include "VN.h"
#include "RVLMotionCommon.h"
#include "Touch.h"

//#define RVLMOTION_TOUCH_VN

using namespace RVL;

Solid::Solid()
{
    vertices.n = 0;
    vertices.Element = NULL;
    edges.n = 0;
    edges.Element = NULL;
    faces.n = 0;
    faces.Element = NULL;
}

Solid::~Solid()
{
    Clear();
}

void Solid::Clear()
{
    RVL_DELETE_ARRAY(vertices.Element);
    RVL_DELETE_ARRAY(edges.Element);
    RVL_DELETE_ARRAY(faces.Element);
    for (int iSolid = 0; iSolid < solids.size(); iSolid++)
        delete solids[iSolid];
}

void Solid::Add(Solid* pSolid)
{
    solids.push_back(pSolid);
}

void Solid::Union()
{
    // Union of vertices.

    vertices.n = 0;
    int iSolid;
    for (iSolid = 0; iSolid < solids.size(); iSolid++)
        vertices.n += solids[iSolid]->vertices.n;
    RVL_DELETE_ARRAY(vertices.Element);
    vertices.Element = new SolidVertex[vertices.n];
    SolidVertex* pVertexMem = vertices.Element;
    Solid* pSolid;
    for (iSolid = 0; iSolid < solids.size(); iSolid++)
    {
        pSolid = solids[iSolid];
        memcpy(pVertexMem, pSolid->vertices.Element, pSolid->vertices.n * sizeof(SolidVertex));
        pVertexMem += pSolid->vertices.n;
    }

    // Bounding box.

    InitBoundingBox<float>(&bbox, vertices.Element[0].P);
    int iVertex;
    for (iVertex = 1; iVertex < vertices.n; iVertex++)
        UpdateBoundingBox<float>(&bbox, vertices.Element[iVertex].P);

    // Union of faces.

    faces.n = 0;
    for (iSolid = 0; iSolid < solids.size(); iSolid++)
        faces.n += solids[iSolid]->faces.n;
    //RVL_DELETE_ARRAY(faces.Element);
    //faces.Element = new SolidFace[faces.n];
    //SolidFace* pFaceMem = faces.Element;
    //for (iSolid = 0; iSolid < solids.size(); iSolid++)
    //{
    //    pSolid = solids[iSolid];

    //}
}

void Solid::Create(Array<Array<int>> faces_)
{
    if (solids.size() == 0)
    {
        // Faces.

        faces.n = faces_.n;
        faces.Element = new SolidFace[faces.n];
        int iFace;
        Array<int>* pFaceSrc = faces_.Element;
        SolidFace* pFace = faces.Element;
        float* P1, * P2, * P3;
        float V1[3], V2[3];
        for (iFace = 0; iFace < faces.n; iFace++, pFace++, pFaceSrc++)
        {
            P1 = vertices.Element[pFaceSrc->Element[0]].P;
            P2 = vertices.Element[pFaceSrc->Element[1]].P;
            P3 = vertices.Element[pFaceSrc->Element[2]].P;
            RVLDIF3VECTORS(P2, P1, V1);
            RVLDIF3VECTORS(P3, P2, V2);
            RVLCROSSPRODUCT3(V1, V2, pFace->N);
            RVLNORM3(pFace->N, pFace->area);
            pFace->d = RVLDOTPRODUCT3(pFace->N, P1);
        }

        // Edges.

        int iVertex;
        for (iVertex = 0; iVertex < vertices.n; iVertex++)
            vertices.Element[iVertex].iEdge = -1;
        edges.n = 0;
        for (iFace = 0; iFace < faces.n; iFace++)
            edges.n += faces_.Element[iFace].n;
        edges.Element = new SolidEdge[edges.n];
        int* edgeMap = new int[vertices.n * vertices.n];
        memset(edgeMap, 0xff, vertices.n * vertices.n * sizeof(int));
        SolidEdge* pEdge;
        int iNext;
        int iEdge = 0;
        int iNextVertex;
        int iTwin;
        pFaceSrc = faces_.Element;
        pFace = faces.Element;
        int i;
        SolidEdge* pTwinEdge;
        float dP[3];
        for (iFace = 0; iFace < faces_.n; iFace++, pFaceSrc++, pFace++)
        {
            pFace->iEdge = iEdge;
            for (i = 0; i < pFaceSrc->n; i++)
            {
                iNext = (i + 1) % pFaceSrc->n;
                pEdge = edges.Element + iEdge;
                pEdge->iVertex = faces_.Element[iFace].Element[i];
                if (vertices.Element[pEdge->iVertex].iEdge < 0)
                    vertices.Element[pEdge->iVertex].iEdge = iEdge;
                pEdge->iFace = iFace;
                pEdge->iNext = (i < pFaceSrc->n - 1 ? iEdge + 1 : pFace->iEdge);
                pEdge->iPrev = (i > 0 ? iEdge - 1 : pFace->iEdge + pFaceSrc->n - 1);
                iNextVertex = faces_.Element[iFace].Element[iNext];
                edgeMap[pEdge->iVertex * vertices.n + iNextVertex] = iEdge;
                iTwin = edgeMap[iNextVertex * vertices.n + pEdge->iVertex];
                if (iTwin >= 0)
                {
                    pEdge->iTwin = iTwin;
                    pTwinEdge = edges.Element + iTwin;
                    pTwinEdge->iTwin = iEdge;
                    pEdge->len = pTwinEdge->len;
                }
                else
                {
                    P1 = vertices.Element[pEdge->iVertex].P;
                    P2 = vertices.Element[iNextVertex].P;
                    RVLDIF3VECTORS(P2, P1, dP);
                    pEdge->len = sqrt(RVLDOTPRODUCT3(dP, dP));
                }
                iEdge++;
            }
        }
        delete[] edgeMap;
    }
}

void Solid::Move(
    Solid* pSolidSrc,
    Pose3D* pMove)
{
    // Parts.

    Solid* pPartSrc;
    Solid* pPart;
    for (int iPart = 0; iPart < solids.size(); iPart++)
    {
        pPart = solids[iPart];
        pPartSrc = pSolidSrc->solids[iPart];
        pPart->Move(pPartSrc, pMove);
    }

    // Vertices.

    SolidVertex* pVertexSrc = pSolidSrc->vertices.Element;
    SolidVertex* pVertex = vertices.Element;
    for (int iVertex = 0; iVertex < vertices.n; iVertex++, pVertex++, pVertexSrc++)
        RVLTRANSF3(pVertexSrc->P, pMove->R, pMove->t, pVertex->P);

    // Faces.

    SolidFace* pFaceSrc = pSolidSrc->faces.Element;
    SolidFace* pFace = faces.Element;
    for (int iFace = 0; iFace < faces.n; iFace++, pFace++, pFaceSrc++)
        RVLPLANETRANSF3(pFaceSrc->N, pFaceSrc->d, pMove->R, pMove->t, pFace->N, pFace->d);
}

void Solid::Copy(Solid* pSolidSrc)
{
    vertices.n = pSolidSrc->vertices.n;
    if (pSolidSrc->vertices.Element != NULL && pSolidSrc->vertices.n > 0)
    {
        vertices.Element = new SolidVertex[vertices.n];
        memcpy(vertices.Element, pSolidSrc->vertices.Element, vertices.n * sizeof(SolidVertex));
    }

    edges.n = pSolidSrc->edges.n;
    if (pSolidSrc->edges.Element != NULL && pSolidSrc->edges.n > 0)
    {
        edges.Element = new SolidEdge[edges.n];
        memcpy(edges.Element, pSolidSrc->edges.Element, edges.n * sizeof(SolidEdge));
    }

    faces.n = pSolidSrc->faces.n;
    if (pSolidSrc->faces.Element != NULL && pSolidSrc->faces.n > 0)
    {
        faces.Element = new SolidFace[faces.n];
        memcpy(faces.Element, pSolidSrc->faces.Element, faces.n * sizeof(SolidFace));
    }

    Solid* pPartSrc, *pPart;
    for (int iSolid = 0; iSolid < pSolidSrc->solids.size(); iSolid++)
    {
        pPartSrc = pSolidSrc->solids[iSolid];
        pPart = new Solid;
        pPart->Copy(pPartSrc);
        solids.push_back(pPart);
    }
}

void Solid::ComputeFaceParams()
{
    float* P1, * P2, * P3;
    float V1[3], V2[3];
    SolidFace* pFace = faces.Element;
    SolidEdge* pEdge;
    for (int iFace = 0; iFace < faces.n; iFace++, pFace++)
    {
        pEdge = edges.Element + pFace->iEdge;
        P1 = vertices.Element[pEdge->iVertex].P;
        pEdge = edges.Element + pEdge->iNext;
        P2 = vertices.Element[pEdge->iVertex].P;
        pEdge = edges.Element + pEdge->iNext;
        P3 = vertices.Element[pEdge->iVertex].P;
        RVLDIF3VECTORS(P2, P1, V1);
        RVLDIF3VECTORS(P3, P2, V2);
        RVLCROSSPRODUCT3(V1, V2, pFace->N);
        RVLNORM3(pFace->N, pFace->area);
        pFace->d = RVLDOTPRODUCT3(pFace->N, P1);
    }
}

Solid *Solid::CreateBox(
    float *size,
    Pose3D* pPose)
{
    // Vertices.

    float halfSize[3];
    RVLSCALE3VECTOR(size, 0.5f, halfSize);
    Vector3<float> vertices_B[8];
    Vector3<float>* pVertex_B = vertices_B;
    RVLSET3VECTOR(pVertex_B->Element, halfSize[0], halfSize[1], -halfSize[2]); pVertex_B++;
    RVLSET3VECTOR(pVertex_B->Element, halfSize[0], -halfSize[1], -halfSize[2]); pVertex_B++;
    RVLSET3VECTOR(pVertex_B->Element, -halfSize[0], -halfSize[1], -halfSize[2]); pVertex_B++;
    RVLSET3VECTOR(pVertex_B->Element, -halfSize[0], halfSize[1], -halfSize[2]); pVertex_B++;
    RVLSET3VECTOR(pVertex_B->Element, halfSize[0], halfSize[1], halfSize[2]); pVertex_B++;
    RVLSET3VECTOR(pVertex_B->Element, -halfSize[0], halfSize[1], halfSize[2]); pVertex_B++;
    RVLSET3VECTOR(pVertex_B->Element, -halfSize[0], -halfSize[1], halfSize[2]); pVertex_B++;
    RVLSET3VECTOR(pVertex_B->Element, halfSize[0], -halfSize[1], halfSize[2]);
    vertices.n = 8;
    vertices.Element = new SolidVertex[8];
    SolidVertex* pVertex = vertices.Element;
    pVertex_B = vertices_B;
    for (int iVertex = 0; iVertex < 8; iVertex++, pVertex++, pVertex_B++)
        RVLTRANSF3(pVertex_B->Element, pPose->R, pPose->t, pVertex->P);

    // Faces.

    int face[6][4] = { {0, 1, 2, 3}, {4, 5, 6, 7}, {0, 4, 7, 1}, {1, 7, 6, 2}, {2, 6, 5, 3}, {3, 5, 4, 0} };
    Array<Array<int>> faces_;
    faces_.n = 6;
    faces_.Element = new Array<int>[6];
    int* facesMem = new int[6 * 4];
    int* pFacesMem = facesMem;
    int iFace;
    for (iFace = 0; iFace < faces_.n; iFace++, pFacesMem += 4)
    {
        faces_.Element[iFace].n = 4;
        faces_.Element[iFace].Element = pFacesMem;
        memcpy(pFacesMem, face[iFace], 4 * sizeof(int));
    }

    // Create solid.

    Create(faces_);

    //

    delete[] faces_.Element;
    delete[] facesMem;

    return this;
}

// Intersection between a line segment and a convex solid.

bool Solid::Intersect(
    float* P1,
    float* P2,
    float* sOut,
    float* V,
    float& l)
{
    RVLDIF3VECTORS(P2, P1, V);
    l = sqrt(RVLDOTPRODUCT3(V, V));
    RVLSCALE3VECTOR2(V, l, V);

    int iFace;
    SolidFace* pFace;
    float s;
    float c, e;
    bool bIntersection[2];
    bIntersection[0] = bIntersection[1] = false;
    for (iFace = 0; iFace < faces.n; iFace++)
    {
        pFace = faces.Element + iFace;

        c = RVLDOTPRODUCT3(pFace->N, V);
        e = RVLDOTPRODUCT3(pFace->N, P1) - pFace->d;

        if (c < -1e-6)
        {
            s = -e / c;
            if (bIntersection[0])
            {
                if (s > sOut[0])
                {
                    sOut[0] = s;
                    if (bIntersection[1])
                    {
                        if (sOut[0] >= sOut[1])
                            return false;
                    }
                }
            }
            else
            {
                sOut[0] = s;
                bIntersection[0] = true;
            }
        }
        else if (c <= 1e-6)
        {
            if (e > 1e-6)
                return false;
        }
        else
        {
            s = -e / c;
            if (bIntersection[1])
            {
                if (s < sOut[1])
                    sOut[1] = s;
                if (bIntersection[0])
                {
                    if (sOut[1] <= sOut[0])
                        return false;
                }
            }
            else
            {
                sOut[1] = s;
                bIntersection[1] = true;
            }
        }
    }

    return true;
}

bool Solid::Intersect(
    Solid* pSolid,
    Array<Pair<Vector3<float>, Vector3<float>>> *pIntersectionsWithOtherEdges,
    Array<Pair<Vector3<float>, Vector3<float>>> *pIntersectionsWithThisEdges)
{
    pIntersectionsWithOtherEdges->n = 0;
    if(pIntersectionsWithThisEdges)
        pIntersectionsWithThisEdges->n = 0;
    IntersectWithConvex(pSolid, pIntersectionsWithOtherEdges, pIntersectionsWithThisEdges);

    Array<Pair<Vector3<float>, Vector3<float>>> intersectionsWithOtherEdges_;
    Array<Pair<Vector3<float>, Vector3<float>>> intersectionsWithThisEdges_;
    Solid* pPart;
    for (int iPart = 0; iPart < pSolid->solids.size(); iPart++)
    {
        pPart = pSolid->solids[iPart];
        intersectionsWithOtherEdges_.Element = pIntersectionsWithOtherEdges->Element + pIntersectionsWithOtherEdges->n;
        intersectionsWithOtherEdges_.n = 0;
        if (pIntersectionsWithThisEdges)
        {
            intersectionsWithThisEdges_.Element = pIntersectionsWithThisEdges->Element + pIntersectionsWithThisEdges->n;
            intersectionsWithThisEdges_.n = 0;
            IntersectWithConvex(pPart, &intersectionsWithOtherEdges_, &intersectionsWithThisEdges_);
        }
        else
            IntersectWithConvex(pPart, &intersectionsWithOtherEdges_);
        pIntersectionsWithOtherEdges->n += intersectionsWithOtherEdges_.n;
        if(pIntersectionsWithThisEdges)
            pIntersectionsWithThisEdges->n += intersectionsWithThisEdges_.n;
    }

    bool bIntersection = (pIntersectionsWithOtherEdges->n > 0);
    if(pIntersectionsWithThisEdges)
        bIntersection = (bIntersection || (pIntersectionsWithThisEdges->n > 0));

    return bIntersection;
}

bool Solid::IntersectWithConvex(
    Solid* pSolid,
    Array<Pair<Vector3<float>, Vector3<float>>> *pIntersectionsWithOtherEdges,
    Array<Pair<Vector3<float>, Vector3<float>>> *pIntersectionsWithThisEdges)
{
    // Src - solid with which the edges of Tgt are intersecting.

    Solid* pSrc = this;
    Solid* pTgt = pSolid;
    Array<Pair<Vector3<float>, Vector3<float>>>* pIntersections = pIntersectionsWithOtherEdges;
    int iEdge;   
    SolidEdge* pEdge;
    float s[2];
    float* P1, * P2;
    float* P1_, * P2_;
    float V[3], V_[3];
    float l;
    Pair<Vector3<float>, Vector3<float>>* pIntersection;
    for (int i = 0; i < 2; i++, pSrc = pSolid, pTgt = this, pIntersections = pIntersectionsWithThisEdges)
    {
        if (pIntersections == NULL)
            break;
        pEdge = pTgt->edges.Element;
        pIntersection = pIntersections->Element;
        for (iEdge = 0; iEdge < pTgt->edges.n; iEdge++, pEdge++)
        {
            if (pEdge->iTwin >= 0 && iEdge > pEdge->iTwin)
                continue;
            P1 = pTgt->vertices.Element[pEdge->iVertex].P;
            P2 = pTgt->vertices.Element[pTgt->edges.Element[pEdge->iNext].iVertex].P;
            if (!pSrc->Intersect(P1, P2, s, V, l))
                continue;
            if (s[1] > 0.0f)
            {
                if (s[1] > l)
                    s[1] = l;
                if (s[0] < l)
                {
                    if (s[0] < 0.0f)
                        s[0] = 0.0f;
                    P1_ = pIntersection->a.Element;
                    RVLSCALE3VECTOR(V, s[0], V_);
                    RVLSUM3VECTORS(P1, V_, P1_);
                    P2_ = pIntersection->b.Element;
                    RVLSCALE3VECTOR(V, s[1], V_);
                    RVLSUM3VECTORS(P1, V_, P2_);
                    pIntersection++;
                }
            }
        }
        pIntersections->n = pIntersection - pIntersections->Element;
    }

    bool bIntersection = (pIntersectionsWithOtherEdges->n > 0);
    if (pIntersectionsWithThisEdges)
        bIntersection = (bIntersection || (pIntersectionsWithThisEdges->n > 0));

    return bIntersection;
}

//#define RVLSOLID_FREEMOVE_VISUALIZE

float Solid::FreeMove(
    float* V,
    Solid* pObstacle)
{
    /// Check collision with obstacle edges. 

#ifdef RVLSOLID_FREEMOVE_VISUALIZE
    RVLCOLORS
    Visualize(pVisualizer, green);
    //pVisualizer->Run();
#endif

    int nObstacleEdges = pObstacle->edges.n;
    int iSolid;
    Solid* pSolid;
    for (iSolid = 0; iSolid < pObstacle->solids.size(); iSolid++)
    {
        pSolid = pObstacle->solids[iSolid];
        nObstacleEdges += pSolid->edges.n;
    }
    nObstacleEdges /= 2;
    Array<Pair<Vector3<float>, Vector3<float>>> obstacleEdgeIntersections;
    obstacleEdgeIntersections.Element = new Pair<Vector3<float>, Vector3<float>>[nObstacleEdges];
    //Array<Pair<Vector3<float>, Vector3<float>>> thisEdgeIntersections;
    //thisEdgeIntersections.Element = new Pair<Vector3<float>, Vector3<float>>[edges.n * pObstacle->solids.size()];
    Pair<Vector3<float>, Vector3<float>>* pIntersection;

    float tmax = sqrt(RVLDOTPRODUCT3(V, V));
    float t = tmax;
    float U[3];
    RVLSCALE3VECTOR2(V, tmax, U);
    SolidFace* pFace = faces.Element;
    Solid hull;
    int iEdge0, iEdge;
    SolidEdge* pEdge;
    SolidVertex* pVertex;
    SolidVertex* pHullVertex;
    Array<Array<int>> faces_;
    faces_.Element = new Array<int>[2 + vertices.n];
    Array<int>* pFace_;
    int* facesMem = new int[(2 + 4) * vertices.n];
    int* pFaceMem;
    int i, j;
    int n;
    float* P[2];
    float t_;
    Array<int> contactVertices;
    contactVertices.Element = new int[vertices.n];
    contactVertices.n = 0;
    bool* bContact = new bool[vertices.n];
    memset(bContact, 0, vertices.n * sizeof(bool));
    float c;
    for (int iFace = 0; iFace < faces.n; iFace++, pFace++)
    {
        c = RVLDOTPRODUCT3(U, pFace->N);
        if (c <= 1e-6)
            continue;

        // Create a motion hull corresponding to face iFace.

        hull.vertices.Element = new SolidVertex[2 * vertices.n];
        pHullVertex = hull.vertices.Element;
        iEdge0 = iEdge = pFace->iEdge;
        do
        {
            pEdge = edges.Element + iEdge;
            pVertex = vertices.Element + pEdge->iVertex;
            RVLCOPY3VECTOR(pVertex->P, pHullVertex->P);
            pHullVertex++;
            RVLSUM3VECTORS(pVertex->P, V, pHullVertex->P);
            pHullVertex++;
            if (!bContact[pEdge->iVertex])
            {
                bContact[pEdge->iVertex] = true;
                contactVertices.Element[contactVertices.n++] = pEdge->iVertex;
            }
            iEdge = pEdge->iNext;
        } while (iEdge != iEdge0);
        hull.vertices.n = pHullVertex - hull.vertices.Element;
        n = hull.vertices.n / 2;
        faces_.n = 2 + n;
        pFace_ = faces_.Element;
        pFace_->n = n;
        pFaceMem = facesMem;
        pFace_->Element = pFaceMem;
        for (i = 2 * (n - 1); i >= 0; i -= 2)
            *(pFaceMem++) = i;
        pFace_++;
        pFace_->n = n;
        pFace_->Element = pFaceMem;
        for (i = 1; i < hull.vertices.n; i += 2)
            *(pFaceMem++) = i;
        pFace_++;
        for (i = 0; i < n; i++)
        {
            pFace_->n = 4;
            pFace_->Element = pFaceMem;
            *(pFaceMem++) = 2 * i;
            *(pFaceMem++) = (2 * i + 2) % hull.vertices.n;
            *(pFaceMem++) = (2 * i + 3) % hull.vertices.n;
            *(pFaceMem++) = 2 * i + 1;
            pFace_++;
        }
        hull.Create(faces_);
        
#ifdef RVLSOLID_FREEMOVE_VISUALIZE
        hull.Visualize(pVisualizer, yellow);
#endif

        // Determine the largest collision-free distance from the starting position.
      
        hull.Intersect(pObstacle, &obstacleEdgeIntersections);

#ifdef RVLSOLID_FREEMOVE_VISUALIZE
        RVLVISUALIZER_LINES_INIT(visPts, visLines, obstacleEdgeIntersections.n)
#endif
        
        for (i = 0; i < obstacleEdgeIntersections.n; i++)
        {
            pIntersection = obstacleEdgeIntersections.Element + i;
            P[0] = pIntersection->a.Element;
            P[1] = pIntersection->b.Element;
            for (j = 0; j < 2; j++)
            {
                t_ = (RVLDOTPRODUCT3(pFace->N, P[j]) - pFace->d) / c;
                if (t_ > 0.0f && t_ < t)
                    t = t_;
            }

#ifdef RVLSOLID_FREEMOVE_VISUALIZE
            RVLCOPY3VECTOR(P[0], visPts.Element[2 * i].P);
            RVLCOPY3VECTOR(P[1], visPts.Element[2 * i + 1].P);
            visLines.Element[i].a = 2 * i;
            visLines.Element[i].b = 2 * i + 1;
#endif
        }
#ifdef RVLSOLID_FREEMOVE_VISUALIZE
        pVisualizer->DisplayLines(visPts, visLines, magenta, 2.0f);
        //pVisualizer->Run();

        RVLVISUALIZER_LINES_FREE(visPts, visLines)
#endif

        hull.Clear();
    }
    delete[] hull.vertices.Element;
    delete[] faces_.Element;
    delete[] facesMem;
    delete[] obstacleEdgeIntersections.Element;
    //delete[] thisEdgeIntersections.Element;
    delete[] bContact;

#ifdef RVLSOLID_FREEMOVE_VISUALIZE
    pVisualizer->Run();
    //pVisualizer->Clear();
#endif

    /// Check collision with obstacle faces.

    float P2[3];
    float t__[2];
    float V3Tmp[3];
    float fTmp;
    int iPart;
    Solid* pPart;
    for (int iContactVertex = 0; iContactVertex < contactVertices.n; iContactVertex++)
    {
        P[0] = vertices.Element[contactVertices.Element[iContactVertex]].P;
        RVLSUM3VECTORS(P[0], V, P2);
        for (iPart = 0; iPart < pObstacle->solids.size(); iPart++)
        {
            pPart = pObstacle->solids[iPart];
            if (!pPart->Intersect(P[0], P2, t__, V3Tmp, fTmp))
                continue;
            if (t__[0] > 0.0f && t__[0] < t)
                t = t__[0];
        }
    }

    ///

    delete[] contactVertices.Element; 
    
    return t;
}

std::vector<vtkSmartPointer<vtkActor>> Solid::Visualize(
    Visualizer* pVisualizer,
    uchar *color)
{
    std::vector<vtkSmartPointer<vtkActor>> actors;
    std::vector<vtkSmartPointer<vtkActor>> partActors;
    for (int iSolid = 0; iSolid < solids.size(); iSolid++)
    {
        partActors = solids[iSolid]->Visualize(pVisualizer, color);
        actors.insert(actors.end(), partActors.begin(), partActors.end());
    }
    if (edges.n > 0)
    {
        RVLVISUALIZER_LINES_INIT(visPts, visLines, edges.n);
        Point* pPt = visPts.Element;
        int iVertex;
        for (iVertex = 0; iVertex < vertices.n; iVertex++, pPt++)
            RVLCOPY3VECTOR(vertices.Element[iVertex].P, pPt->P);
        visPts.n = pPt - visPts.Element;
        Pair<int, int>* pLine = visLines.Element;
        SolidEdge* pEdge = edges.Element;
        int iEdge;
        for (iEdge = 0; iEdge < edges.n; iEdge++, pEdge++, pLine++)
        {
            pLine->a = pEdge->iVertex;
            pLine->b = edges.Element[pEdge->iNext].iVertex;
        }
        actors.push_back(pVisualizer->DisplayLines(visPts, visLines, color));
        RVLVISUALIZER_LINES_FREE(visPts, visLines);
    }
    if (vertices.n > 0)
    {
        Array<Point> visPts;
        Point visPt;
        RVLCOPY3VECTOR(vertices.Element[0].P, visPt.P);
        visPts.n = 1;
        visPts.Element = &visPt;
        actors.push_back(pVisualizer->DisplayPointSet<float, Point>(visPts, color, 6.0f));
    }

    return actors;
}

Touch::Touch()
{
    bDoor = false;
    //kappa = 1.0f / (0.00000285f * 0.7f * 1000.0f);
    kappa = 1.0f;
    zn = 1.0f;
    stdgxyk = 0.05f;
    stdgzk = 0.02f;
    stdhxyk = 0.02f;
    stdhzk = 0.08f;
    stdphi = 3.0f;  // deg
    stds = 0.01f;   // m
    stdc = 0.01f;   // m
    alpha = 1e-6;
    beta = 1e-3;
    contactAngleThr = 85.0f;
    maxReconstructionError = 0.05f; // m
    toolTiltDeg = 20.0f;    // deg
    nSamples = 100;
    float xInc_[15] = { 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 1e-3, 1e-3, 1e-3, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4 };
    memcpy(xInc, xInc_, 15 * sizeof(float));
    bRefPtConstraints = false;
    simulationSeed = 0;
    nSimulationTouches = 10;
    model_gt.pVNEnv = NULL;
    model_gt.d = NULL;
    model_gt.plane = NULL;
    model_gt.vertex = NULL;
    model_e.pVNEnv = NULL;
    model_e.d = NULL;
    model_e.plane = NULL;
    model_e.vertex = NULL;
    model_x.pVNEnv = NULL;
    model_x.d = NULL;
    model_x.plane = NULL;
    model_x.vertex = NULL;
    rndVal.Element = NULL;
    VNMClusters.Element = NULL;
    pVisualizationData = NULL;
    SDFBuff = NULL;
    surfaces.Element = NULL;
    surfaceMem = NULL;
    vertices.Element = NULL;
    vertexMem = NULL;
    tool.pVN = NULL;
    tool.VNMClusters.Element = NULL;
    tool.d = NULL;
    solidVNAssoc = NULL;
    refPts.n = 1;
    refPts.Element = NULL;

    Constants();
}

Touch::~Touch()
{
    Clear();
}

void Touch::Create(char* cfgFileName)
{
    // Load paramters from a configuration file.

    CreateParamList();
    paramList.LoadParams(cfgFileName);

    // Load camera parameters from the configuration file.

    LoadCameraParametersFromFile(cfgFileName, camera, pMem0);

    // Constants.

    Constants();

    // Random indices.

    RVL_DELETE_ARRAY(rndVal.Element);
    rndVal.n = 1000000;
    RandomIndices(rndVal);
}

void Touch::CreateParamList()
{
    paramList.m_pMem = pMem0;
    RVLPARAM_DATA* pParamData;
    paramList.Init();
    pParamData = paramList.AddParam("Touch.stdgxyk", RVLPARAM_TYPE_FLOAT, &stdgxyk);
    pParamData = paramList.AddParam("Touch.stdgzk", RVLPARAM_TYPE_FLOAT, &stdgzk);
    pParamData = paramList.AddParam("Touch.stdhxyk", RVLPARAM_TYPE_FLOAT, &stdhxyk);
    pParamData = paramList.AddParam("Touch.stdhzk", RVLPARAM_TYPE_FLOAT, &stdhzk);
    pParamData = paramList.AddParam("Touch.stdphi", RVLPARAM_TYPE_FLOAT, &stdphi);
    pParamData = paramList.AddParam("Touch.stds", RVLPARAM_TYPE_FLOAT, &stds);
    pParamData = paramList.AddParam("Touch.alpha", RVLPARAM_TYPE_FLOAT, &alpha);
    pParamData = paramList.AddParam("Touch.nSamples", RVLPARAM_TYPE_INT, &nSamples);
    pParamData = paramList.AddParam("Touch.Simulation_seed", RVLPARAM_TYPE_INT, &simulationSeed);
    pParamData = paramList.AddParam("Touch.Simulation_num_touches", RVLPARAM_TYPE_INT, &nSimulationTouches);
}

void Touch::Clear()
{
    DeleteTouchModel(&model_gt);
    DeleteTouchModel(&model_e);
    DeleteTouchModel(&model_x);
    RVL_DELETE_ARRAY(VNMClusters.Element);
    RVL_DELETE_ARRAY(SDFBuff);
    RVL_DELETE_ARRAY(surfaces.Element);
    RVL_DELETE_ARRAY(surfaceMem);
    RVL_DELETE_ARRAY(vertices.Element);
    RVL_DELETE_ARRAY(vertexMem);
    RVL_DELETE_ARRAY(tool.VNMClusters.Element);
    if (tool.pVN)
        delete tool.pVN;
    RVL_DELETE_ARRAY(tool.d);
    RVL_DELETE_ARRAY(solidVNAssoc);
    RVL_DELETE_ARRAY(refPts.Element);
}

void Touch::LM(
    float* x0,
    Array<MOTION::TouchData> touches,
    float *x,
    MOTION::TouchLMError* pErr)
{
    RVLCOLORS

    // Parameters.

    int maxnIterations = 100;
    float gamma = 1.5f;

    // x <- x0

    memcpy(x, x0, RVLMOTION_TOUCH_NUM_PARAMS * sizeof(float));

    // Initial damping factor.

    float lm = 1e-3;

    // Initial constraint factor.

    float rho = 1.0f;

    /// Main loop.

    int it;
    int iTouch;    
    cv::Mat cvJ(touches.n, RVLMOTION_TOUCH_NUM_PARAMS, CV_64FC1);
    double* J = (double*)(cvJ.data);
    cv::Mat cvr(touches.n, 1, CV_64FC1);
    double *r = (double*)(cvr.data);
    cv::Mat cvQ(RVLMOTION_TOUCH_NUM_PARAMS, RVLMOTION_TOUCH_NUM_PARAMS, CV_64FC1);
    double* Q = (double*)(cvQ.data);
    cv::Mat cvR(RVLMOTION_TOUCH_NUM_PARAMS, 1, CV_64FC1);
    double* R = (double*)(cvR.data);
    cv::Mat cvdx(RVLMOTION_TOUCH_NUM_PARAMS, 1, CV_64FC1);
    double *dx = (double*)(cvdx.data);
    cv::Mat cvJg(touches.n, RVLMOTION_TOUCH_NUM_PARAMS, CV_64FC1);
    double* Jg = (double*)(cvJg.data);
    cv::Mat cvg(touches.n, 1, CV_64FC1);
    double* g = (double*)(cvg.data);
    cv::Mat cvJh(refPts.n, RVLMOTION_TOUCH_NUM_PARAMS, CV_64FC1);
    double* Jh = (double*)(cvJh.data);
    cv::Mat cvh(refPts.n, 1, CV_64FC1);
    double* h = (double*)(cvh.data);
    int iRefPt;
    Array<int> ig;
    ig.Element = new int[touches.n];
    bool* bg = new bool[touches.n];
    int i;
    float xPrev[RVLMOTION_TOUCH_NUM_PARAMS];
    float oldx;
    float E, EPrev, Er, Eg, Ex, Eh, rmax, absr;
    float gmax = 0.0f;
    float ravg, Egavg; // Only for testig purpose.
    MOTION::TouchData* pTouch;
    Pair<float, float> err;
    int iConstraint;
    float dr;
    float dg;
    float dh;
    int nTouchesWithContacts;     // Only for testig purpose.
    for (it = 0; it < maxnIterations; it++)
    {
        // Residuals.

        UpdateEnvironmentModel(&model_e, x, &model_x);
#ifdef RVLMOTION_TOUCH_VN
        model_x.pVNEnv->CopyDescriptor(model_x.d);
#endif
        if (pVisualizationData->bOptimization)
        {
            pVisualizationData->pVisualizer->Clear(pVisualizationData->errorActors);
            pVisualizationData->errorActors.clear();
        }
        ig.n = 0;
        nTouchesWithContacts = 0;     // Only for testig purpose.
        for (iTouch = 0; iTouch < touches.n; iTouch++)
        {
            pTouch = touches.Element + iTouch;
#ifdef RVLMOTION_TOUCH_VN
            r[iTouch] = SDF(pTouch, model_x.pVNEnv, model_x.d);
#else
            if (pTouch->nContacts > 0)
            {
                nTouchesWithContacts++;   // Only for testig purpose.
                err = Error(pTouch);
                r[iTouch] = err.a;
                if (bg[iTouch] = (err.b > 1e-4))
                {
                    g[ig.n] = err.b;
                    ig.Element[ig.n++] = iTouch;
                }
            }
            else
            {
                r[iTouch] = 0.0f;
                bg[iTouch] = false;
            }
#endif
        }

        if (bRefPtConstraints)
        {
            for (iRefPt = 0; iRefPt < refPts.n; iRefPt++)
            {
                pTouch = refPts.Element + iRefPt;
                err = Error(pTouch, true, true);
                h[iRefPt] = err.a;
            }
        }

        if (pVisualizationData->bOptimization)
        {
            pVisualizationData->pVisualizer->Clear(pVisualizationData->envActors);
            pVisualizationData->envActors = envSolidx.Visualize(pVisualizationData->pVisualizer, green);
            pVisualizationData->pVisualizer->Run();
        }

        // Cost.

        Er = 0.0f;
        rmax = 0.0f;
        for (iTouch = 0; iTouch < touches.n; iTouch++)
        {
            Er += r[iTouch] * r[iTouch];
            absr = RVLABS(r[iTouch]);
            if (absr > rmax)
                rmax = absr;
        }
        ravg = sqrt(Er / (float)nTouchesWithContacts);  // Only for testig purpose.
        Eg = 0.0f;
        if (ig.n > 0)
        {
            for (iConstraint = 0; iConstraint < ig.n; iConstraint++)
            {
                Eg += rho * g[iConstraint] * g[iConstraint];
                if (iConstraint == 0 || g[iConstraint] > gmax)
                    gmax = g[iConstraint];
            }
            Egavg = sqrt(Eg / (float)ig.n); // Only for testig purpose.
        }
        else
            Egavg = gmax = 0.0f; // Only for testig purpose.
        Ex = 0.0f;
        Eh = 0.0f;
        if (bRefPtConstraints)
        {
            for (iRefPt = 0; iRefPt < refPts.n; iRefPt++)
                Eh += h[iRefPt] * h[iRefPt];
            Eh *= beta;
        }
        else
        {
            for (i = 0; i < RVLMOTION_TOUCH_NUM_PARAMS; i++)
                Ex += x[i] * x[i] / varx[i];
            Ex *= alpha;
        }
        E = Er + Eg + Ex + Eh;
        //printf("E=%f rmax=%f gmax=%f\n", E, rmax, gmax);

        // Check convergence.

        if (it > 0)
        {
            if (E <= EPrev)
            {
                lm *= 0.1f;
                for (i = 0; i < RVLMOTION_TOUCH_NUM_PARAMS; i++)
                    if (RVLABS(dx[i]) > xInc[i])
                        break;
                if (i >= RVLMOTION_TOUCH_NUM_PARAMS)
                    break;
                EPrev = E;
            }
            else
            {
                memcpy(x, xPrev, RVLMOTION_TOUCH_NUM_PARAMS * sizeof(float));
                lm *= 10.0f;
            }
        }
        else
            EPrev = E;

        // Jacobian.

        memset(Jg, 0, touches.n * RVLMOTION_TOUCH_NUM_PARAMS * sizeof(double));
        for (i = 0; i < RVLMOTION_TOUCH_NUM_PARAMS; i++)
        {
            oldx = x[i];
            x[i] = oldx + xInc[i];
            UpdateEnvironmentModel(&model_e, x, &model_x);
            model_x.pVNEnv->CopyDescriptor(model_x.d);
            iConstraint = 0;
            for (iTouch = 0; iTouch < touches.n; iTouch++)
            {
                pTouch = touches.Element + iTouch;
#ifdef RVLMOTION_TOUCH_VN
                J[iTouch * RVLMOTION_TOUCH_NUM_PARAMS + i] = (SDF(pTouch, model_x.pVNEnv, model_x.d) - r[iTouch]) / xInc[i];
#else
                if (pTouch->nContacts > 0)
                {
                    err = Error(pTouch);
                    dr = (err.a - r[iTouch]) / xInc[i];
                }
                else
                    dr = 0.0f;
                J[iTouch * RVLMOTION_TOUCH_NUM_PARAMS + i] = dr;
                if (bg[iTouch])
                {
                    dg = (err.b - g[iConstraint]) / xInc[i];
                    Jg[iConstraint * RVLMOTION_TOUCH_NUM_PARAMS + i] = dg;
                    iConstraint++;
                }
#endif
            }
            if (bRefPtConstraints)
            {
                for (iRefPt = 0; iRefPt < refPts.n; iRefPt++)
                {
                    pTouch = refPts.Element + iRefPt;
                    err = Error(pTouch, true, true);
                    dh = (err.a - h[iRefPt]) / xInc[i];
                    Jh[iRefPt * RVLMOTION_TOUCH_NUM_PARAMS + i] = dh;
                }
            }
            x[i] = oldx;
        }
        
        // xnew <- solve ((J.T * J + Jg.T * Jg + alpha * C.inv + lm * I) * dx_  = -(J.T * r + Jg.T * g + alpha * C.inv * x))

        cvQ = cvJ.t() * cvJ;
        cvR = -cvJ.t() * cvr;
        if (ig.n > 0)
        {
            cvQ += rho * cvJg.t() * cvJg;
            cvR -= rho * cvJg.t() * cvg;
        }
        if (bRefPtConstraints)
        {
            cvQ += beta * cvJh.t() * cvJh;
            cvR -= beta * cvJh.t() * cvh;
        }
        else
        {
            for (i = 0; i < RVLMOTION_TOUCH_NUM_PARAMS; i++)
            {
                Q[i * RVLMOTION_TOUCH_NUM_PARAMS + i] += (alpha / varx[i]);
                R[i] -= alpha * x[i] / varx[i];
            }
        }
        for (i = 0; i < RVLMOTION_TOUCH_NUM_PARAMS; i++)
            Q[i * RVLMOTION_TOUCH_NUM_PARAMS + i] += lm;
        cv::solve(cvQ, cvR, cvdx);
        memcpy(xPrev, x, RVLMOTION_TOUCH_NUM_PARAMS * sizeof(float));
        RVLSUMVECTORS(x, dx, RVLMOTION_TOUCH_NUM_PARAMS, x, i);
    }
    delete[] ig.Element;
    delete[] bg;

    if (pVisualizationData->bOptimization)
    {
        pVisualizationData->pVisualizer->Clear(pVisualizationData->errorActors);
        pVisualizationData->pVisualizer->Clear(pVisualizationData->envActors);
    }

    pErr->E = E;
    pErr->ravg = ravg;
    pErr->gmax = gmax;
    pErr->Ex = Ex;
    //printf("Er=%f Eravg=%f Eg=%f Egavg=%f Ex=%f E=%f\n", Er, Eravg, Eg, Egavg, Ex, E);    
}

float Touch::SDF(
    MOTION::TouchData* pTouchData,
    VN* pVNEnv,
    float *d)
{
    MOTION::PlanarSurface* pSurface = surfaces.Element + pTouchData->contact.iEnvFeature.a;
    RECOG::VN_::Feature* pFeature = pVNEnv->featureArray.Element + pSurface->VNFeatures.Element[0];
    float e = RVLDOTPRODUCT3(pFeature->N, pTouchData->pose.t) - pFeature->d;

    //int iActiveFeature;
    //return pVNEnv->Evaluate(pToolPose->t, SDFBuff, iActiveFeature, d);

    return e;
}

Pair<float, float> Touch::Error(
    MOTION::TouchData* pTouchData,
    bool bToolPositioned,
    bool bPointTool)
{
    RVLCOLORS
    Array<Point> visPts;
    Point visPtMem[2];
    //Array<Pair<int, int>> visLines;
    //Pair<int, int> visLine;
    if (pVisualizationData->bOptimization)
    {
        visPts.Element = visPtMem;
        visPts.n = 2;
        //visLines.Element = &visLine;
        //visLines.n = 1;
        //visLine.a = 0;
        //visLine.b = 1;
    }

    Pair<float, float> e;
    float e_;
    SolidFace* pFace;
    SolidFace* pFace_;
    if(!bToolPositioned)
        toolMoved.Move(&(tool.solid), &(pTouchData->pose));
    if (pTouchData->contact.type == RVLMOTION_TOUCH_CONTACT_TYPE_POINT_PLANE || pTouchData->contact.type == RVLMOTION_TOUCH_CONTACT_TYPE_PLANE_POINT)
    {
        float* P;
        Solid* pSolid;        
        if (pTouchData->contact.type == RVLMOTION_TOUCH_CONTACT_TYPE_POINT_PLANE)
        {
            P = (bPointTool ? pTouchData->pose.t : toolMoved.vertices.Element[pTouchData->contact.iToolFeature].P);
            pSolid = envSolidx.solids[pTouchData->contact.iEnvFeature.a];
            pFace = pSolid->faces.Element + pTouchData->contact.iEnvFeature.b;
        }
        else
        {
            P = envSolidx.solids[pTouchData->contact.iEnvFeature.a]->vertices.Element[pTouchData->contact.iEnvFeature.b].P;
            pSolid = &toolMoved;
            pFace = toolMoved.faces.Element + pTouchData->contact.iToolFeature;
        }
        e.a = RVLDOTPRODUCT3(pFace->N, P) - pFace->d;
        e.b = 0.0f;
        int iEdge0 = pFace->iEdge;
        int iEdge = iEdge0;
        SolidEdge* pEdge;
        do
        {
            pEdge = pSolid->edges.Element + iEdge;
            pFace_ = pSolid->faces.Element + pSolid->edges.Element[pEdge->iTwin].iFace;
            e_ = RVLDOTPRODUCT3(pFace_->N, P) - pFace_->d;
            if (e_ > e.b)
                e.b = e_;
            iEdge = pEdge->iNext;
        } while (iEdge != iEdge0);

        // Visualization.

        if (pVisualizationData->bOptimization)
        {
            RVLCOPY3VECTOR(P, visPts.Element[0].P);
            float V3Tmp[3];
            RVLSCALE3VECTOR(pFace->N, e.a, V3Tmp);
            RVLDIF3VECTORS(P, V3Tmp, visPts.Element[1].P);
            pVisualizationData->envActors.push_back(pVisualizationData->pVisualizer->DisplayLine(visPts.Element, red));
        }
    }
    else if (pTouchData->contact.type == RVLMOTION_TOUCH_CONTACT_TYPE_EDGE_EDGE)
    {
        SolidEdge* pToolEdge = toolMoved.edges.Element + pTouchData->contact.iToolFeature;
        Solid* pEnvSolid = envSolidx.solids[pTouchData->contact.iEnvFeature.a];
        SolidEdge* pEnvEdge = pEnvSolid->edges.Element + pTouchData->contact.iEnvFeature.b;
        float* P11 = toolMoved.vertices.Element[pToolEdge->iVertex].P;
        float* P12 = toolMoved.vertices.Element[toolMoved.edges.Element[pToolEdge->iNext].iVertex].P;
        float* P21 = pEnvSolid->vertices.Element[pEnvEdge->iVertex].P;
        float* P22 = pEnvSolid->vertices.Element[pEnvSolid->edges.Element[pEnvEdge->iNext].iVertex].P;
        float lm1, lm2;
        float V1[3], V2[3], AC[3], BC[3], DCSqrMag, inPlaneA[3], inPlaneB[3], inPlaneBA[3];
        float P1_[3], P2_[3];
        float V3Tmp[3];
        float fTmp;
        RVL3DLINE_SEGMENTS_CLOSEST_POINTS(P11, P12, P21, P22, P1_, P2_, lm1, lm2, V1, V2, AC, BC, DCSqrMag, inPlaneA, inPlaneB, inPlaneBA, fTmp, V3Tmp);
        float N[3];
        RVLCROSSPRODUCT3(V1, V2, N);
        RVLNORM3(N, fTmp);
        float dP_[3];
        RVLDIF3VECTORS(P2_, P1_, dP_);
        e.a = RVLDOTPRODUCT3(dP_, N);
        RVLSCALE3VECTOR(N, e.a, V3Tmp);
        RVLDIF3VECTORS(dP_, V3Tmp, V3Tmp);
        e.b = sqrt(RVLDOTPRODUCT3(V3Tmp, V3Tmp));

        // Visualization.

        if (pVisualizationData->bOptimization)
        {
            RVLCOPY3VECTOR(P1_, visPts.Element[0].P);
            RVLCOPY3VECTOR(P2_, visPts.Element[1].P);
            pVisualizationData->envActors.push_back(pVisualizationData->pVisualizer->DisplayLine(visPts.Element, red));
        }
    }

    return e;
}

// Move to Util.

void ThreePlanesIntersection(
    float* N1, float d1,
    float* N2, float d2,
    float* N3, float d3,
    float* P)
{
    cv::Mat cvM(3, 3, CV_32FC1);
    float* rowM = (float*)(cvM.data);
    RVLCOPY3VECTOR(N1, rowM);
    rowM += 3;
    RVLCOPY3VECTOR(N2, rowM);
    rowM += 3;
    RVLCOPY3VECTOR(N3, rowM);
    cv::Mat cvd(3, 1, CV_32FC1);
    float* d = (float*)(cvd.data);
    RVLSET3VECTOR(d, d1, d2, d3);
    cv::Mat cvP(3, 1, CV_32FC1, P);
    cv::solve(cvM, cvd, cvP);
}

void Touch::Sample(
    int nSamplesIn,
    float rayDist,
    Array<MOTION::TouchSample> &samples)
{
    samples.Element = new MOTION::TouchSample[nSamplesIn];
    samples.n = nSamplesIn;
    int iSample;
    MOTION::TouchSample* pSample;
    int i;
    RECOG::VN_::Feature* pFeature;
    for (iSample = 0; iSample < nSamplesIn; iSample++)
    {
        // Choose a panel.

        int iPanel;
        RVLRND(4, rndVal.Element, 1000000, iRndVal, iPanel);

        // Panel bbox.

        float P_W[3];
        Box<float> panelBBox;
        int j, k;
        int iVertex = 0;
        float N1[3], N2[3], N3[3];
        float d1, d2, d3;
        for (i = 0; i < 2; i++)
        {
            pFeature = model_x.pVNEnv->featureArray.Element + 18 * iPanel + 3 * i;
            RVLPLANETRANSF3(pFeature->N, pFeature->d, pose_E_W.R, pose_E_W.t, N1, d1);
            for (j = 0; j < 2; j++)
            {
                pFeature = model_x.pVNEnv->featureArray.Element + 18 * iPanel + 3 * j + 1;
                RVLPLANETRANSF3(pFeature->N, pFeature->d, pose_E_W.R, pose_E_W.t, N2, d2);
                for (k = 0; k < 2; k++, iVertex++)
                {
                    pFeature = model_x.pVNEnv->featureArray.Element + 18 * iPanel + 3 * k + 2;
                    RVLPLANETRANSF3(pFeature->N, pFeature->d, pose_E_W.R, pose_E_W.t, N3, d3);
                    ThreePlanesIntersection(N1, d1, N2, d2, N3, d3, P_W);
                    if (iVertex == 0)
                        InitBoundingBox<float>(&panelBBox, P_W);
                    else
                        UpdateBoundingBox<float>(&panelBBox, P_W);
                }
            }
        }
        float panelBBoxSize[3];
        BoxSize<float>(&panelBBox, panelBBoxSize[0], panelBBoxSize[1], panelBBoxSize[2]);
        float panelBBoxVertex0[3];
        RVLSET3VECTOR(panelBBoxVertex0, panelBBox.minx, panelBBox.miny, panelBBox.minz);

        // Sample a contact point on the panel surface reconstructed by the vision system.

        int iAxis1;
        float P1_W[3], P2_W[3];
        float P1_E[3], P2_E[3];
        Array<Pair<RECOG::VN_::SurfaceRayIntersection, RECOG::VN_::SurfaceRayIntersection>>* pIntersection;
        float s;
        Pair<RECOG::VN_::SurfaceRayIntersection, RECOG::VN_::SurfaceRayIntersection>* pIntersectionSegment;
        int iIntersectionSegment;
        Array<Pair<float, float>> intersectionPts;
        Pair<float, float> intersectionPtMem[4];
        intersectionPts.Element = intersectionPtMem;
        Pair<float, float>* pIntersectionPt;
        do
        {
            for (i = 0; i < 3; i++)
                P_W[i] = panelBBoxVertex0[i] + panelBBoxSize[i] * RealPseudoRand<float>(rndVal, iRndVal);
            RVLRND(3, rndVal.Element, 1000000, iRndVal, iAxis1);
            RVLCOPY3VECTOR(P_W, P1_W);
            P1_W[iAxis1] = panelBBoxVertex0[iAxis1] - rayDist;
            RVLTRANSF3(P1_W, pose_W_E.R, pose_W_E.t, P1_E);
            RVLCOPY3VECTOR(P_W, P2_W);
            P2_W[iAxis1] = panelBBoxVertex0[iAxis1] + panelBBoxSize[iAxis1] + rayDist;
            RVLTRANSF3(P2_W, pose_W_E.R, pose_W_E.t, P2_E);
            pIntersection = model_x.pVNEnv->VolumeCylinderIntersection(model_x.d, P1_E, P2_E, 0.0f);
            pIntersectionSegment = pIntersection->Element;
            pIntersectionPt = intersectionPts.Element;
            for (iIntersectionSegment = 0; iIntersectionSegment < pIntersection->n; iIntersectionSegment++, pIntersectionSegment++)
            {
                if (pIntersectionSegment->a.s >= 0.5f * rayDist && pIntersectionSegment->a.s <= panelBBoxSize[iAxis1] + 1.5f * rayDist)
                {
                    pIntersectionPt->a = pIntersectionSegment->a.s;
                    pIntersectionPt->b = 1.0f;
                    pIntersectionPt++;
                }
                if (pIntersectionSegment->b.s >= 0.5f * rayDist && pIntersectionSegment->b.s <= panelBBoxSize[iAxis1] + 1.5f * rayDist)
                {
                    pIntersectionPt->a = pIntersectionSegment->b.s;
                    pIntersectionPt->b = -1.0f;
                    pIntersectionPt++;
                }
            }
            intersectionPts.n = pIntersectionPt - intersectionPts.Element;
        } while (intersectionPts.n == 0);
        if (intersectionPts.n == 1)
            j = 0;
        else
            RVLRND(intersectionPts.n, rndVal.Element, 1000000, iRndVal, j);
        s = intersectionPts.Element[j].a;
        P_W[iAxis1] = P1_W[iAxis1] + s;
        pSample = samples.Element + iSample;
        RVLCOPY3VECTOR(P_W, pSample->P);
        pSample->iAxis = iAxis1;
        pSample->direction = intersectionPts.Element[j].b;
    }
}

void Touch::CreateEnvironmentModelTemplete(
    MOTION::TouchModel *pModel,
    int nPanels,
    Array<RECOG::PSGM_::Plane> CT,
    Pair<float, float> betaInterval,
    Pose3D *pPose_Arot_A)
{
    if (pModel->pVNEnv)
        delete pModel->pVNEnv;
    pModel->pVNEnv = new VN;
    pModel->pVNEnv->CreateEmpty();
    float R[9];
    RVLUNITMX3(R);
    float t[3];
    RVLNULL3VECTOR(t);
    Array2D<float> NArray;
    NArray.w = 3;
    NArray.h = 0;
    for (int iPanel = 0; iPanel < nPanels; iPanel++)
    {
        if (bDoor && iPanel == nPanels - 1)
            VNMClusters.Element[iPanel] = pModel->pVNEnv->AddModelCluster(iPanel, RVLVN_CLUSTER_TYPE_CONVEX, pPose_Arot_A->R, t, 0.5f, CT, betaInterval, NArray, pMem0);
        else
            VNMClusters.Element[iPanel] = pModel->pVNEnv->AddModelCluster(iPanel, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.5f, CT, betaInterval, NArray, pMem0);
    }
    pModel->pVNEnv->AddOperation(nPanels, -1, 0, 1, pMem0);
    pModel->pVNEnv->AddOperation(nPanels + 1, -1, 2, 3, pMem0);
    pModel->pVNEnv->AddOperation(nPanels + 2, -1, nPanels, nPanels + 1, pMem0);
    if (bDoor)
        pModel->pVNEnv->AddOperation(nPanels + 3, -1, nPanels + 2, nPanels - 1, pMem0);
    pModel->pVNEnv->SetOutput(nPanels + 3);
    pModel->pVNEnv->Create(pMem0);    
    RVL_DELETE_ARRAY(pModel->d);
    pModel->d = new float[pModel->pVNEnv->featureArray.n];
    RVL_DELETE_ARRAY(pModel->plane);
    pModel->plane = new MOTION::Plane[surfaces.n];
    pModel->vertex = new Vector3<float>[vertices.n];
}

void Touch::DeleteTouchModel(MOTION::TouchModel* pModel)
{
    if(pModel->pVNEnv)
        delete pModel->pVNEnv;
    RVL_DELETE_ARRAY(pModel->d);
    RVL_DELETE_ARRAY(pModel->plane);
    RVL_DELETE_ARRAY(pModel->vertex);
}

#define RVLSET6VECTOR(V, a, b, c, d, e, f)	{V[0] = a; V[1] = b; V[2] = c; V[3] = d; V[4] = e; V[5] = f;}

void Touch::CreateScene(
    float sx,
    float sy,
    float sz,
    float rx,
    float ry,
    float a,
    float b,
    float c,
    float qDeg)
{
    // Constants.

    float q = qDeg * DEG2RAD;

    // Scene.

    int nPanels = 4;
    float scene[4][6];
        //float scene[4][6] = { {0.300f, 0.420f, 0.020f, 0.0f, 0.0f, 0.250f},
        //    {0.300f, 0.420f, 0.020f, 0.0f, 0.0f, -0.250f},
        //    {0.300f, 0.020f, 0.520f, 0.0f, 0.200f, 0.0f},
        //    {0.300f, 0.020f, 0.520f, 0.0f, -0.200f, 0.0f} };

    RVLSET6VECTOR(scene[0], a, sy + 2.0f * (sx + c), sx, 0.0f, 0.0f, 0.5f * (sx + sz) + c);
    RVLSET6VECTOR(scene[1], a, sy + 2.0f * (sx + c), sx, 0.0f, 0.0f, -(0.5f * (sx + sz) + c));
    RVLSET6VECTOR(scene[2], a, sx, sz + 2.0f * (sx + c), 0.0f, 0.5f * (sx + sy) + c, 0.0f);
    RVLSET6VECTOR(scene[3], a, sx, sz + 2.0f * (sx + c), 0.0f, -(0.5f * (sx + sy) + c), 0.0f);

    float* panelSize;
    int i, j;
    int iPanel;
    Solid* pPanel;
    Pose3D pose_B_W;
    RVLUNITMX3(pose_B_W.R);
    float* t_B_W;
    bool bFirstVertex = false;
    for (iPanel = 0; iPanel < nPanels; iPanel++)
    {
        panelSize = scene[iPanel];
        pPanel = new Solid;
        t_B_W = scene[iPanel] + 3;
        RVLCOPY3VECTOR(t_B_W, pose_B_W.t);
        pPanel->CreateBox(panelSize, &pose_B_W);
        envSolid.Add(pPanel);
    }
    //envSolid.Union();
    float bboxSize = BoxSize<float>(&(envSolid.bbox));

    // Door.

    Pose3D pose_Arot_A;
    if (bDoor)
    {
        pPanel = new Solid;
        Pose3D pose_B_Arot;
        RVLUNITMX3(pose_B_Arot.R);
        RVLSET3VECTOR(pose_B_Arot.t, rx, ry, 0.0f);
        float cs = cos(q);
        float sn = sin(q);
        RVLROTZ(cs, sn, pose_Arot_A.R);
        RVLNULL3VECTOR(pose_Arot_A.t);
        RVLUNITMX3(doorPose.R);
        RVLSET3VECTOR(doorPose.t, -0.5f * a, 0.5f * sy + b, 0.0f);
        Pose3D pose_B_A;
        RVLCOMPTRANSF3D(pose_Arot_A.R, pose_Arot_A.t, pose_B_Arot.R, pose_B_Arot.t, pose_B_A.R, pose_B_A.t);
        RVLCOMPTRANSF3D(doorPose.R, doorPose.t, pose_B_A.R, pose_B_A.t, pose_B_W.R, pose_B_W.t);
        float panelSize_[3];
        RVLSET3VECTOR(panelSize_, sx, sy, sz);
        pPanel->CreateBox(panelSize_, &pose_B_W);
        envSolid.Add(pPanel);
        nPanels++;
    }

    // Visualize environment model.

    //RVLCOLORS
    //Visualizer* pVisualizer = pVisualizationData->pVisualizer;
    //if (bDoor)
    //    pVisualizer->DisplayReferenceFrame(&doorPose, 0.2f);
    //envSolid.Visualize(pVisualizer, black);
    //pVisualizer->Run();

    // Panel Surfaces.

    //Array<MOTION::Plane> panelSurfaces;
    //panelSurfaces.n = nPanels * 6;
    //panelSurfaces.Element = new MOTION::Plane[panelSurfaces.n];    
    //MOTION::Plane* pPanelSurface = panelSurfaces.Element;
    //float* P_W__[3];
    //int iPlane = 0;
    //float area;
    //for (iPanel = 0; iPanel < nPanels; iPanel++)
    //{
    //    for (i = 0; i < 6; i++)
    //    {
    //        iPlane = 6 * iPanel + i;
    //        pPanelSurface = panelSurfaces.Element + iPlane;
    //        face_ = face + 2 * iPlane * 3;
    //        for(j = 0; j < 3; j++)
    //            P_W__[j] = P_W + face_[j] * 3;
    //        TriangleNormalAndArea(P_W__, pPanelSurface->N, area);
    //        pPanelSurface->d = RVLDOTPRODUCT3(pPanelSurface->N, P_W__[0]);
    //    }
    //}

    // Surfaces.

    //RVL_DELETE_ARRAY(surfaces.Element);
    //surfaces.Element = new MOTION::PlanarSurface[panelSurfaces.n];
    //MOTION::PlanarSurface *pSurface = surfaces.Element;
    //bool* bJoined = new bool[panelSurfaces.n];
    //memset(bJoined, 0, panelSurfaces.n * sizeof(bool));
    //int iPlane_;
    //MOTION::Plane* pPanelSurface_;
    //float e;
    //for (iPlane = 0; iPlane < panelSurfaces.n; iPlane++)
    //{
    //    if (bJoined[iPlane])
    //        continue;
    //    bJoined[iPlane] = true;
    //    pPanelSurface = panelSurfaces.Element + iPlane;
    //    pSurface->plane = *pPanelSurface;
    //    pSurface++;
    //    for (iPlane_ = iPlane + 1; iPlane_ < panelSurfaces.n; iPlane_++)
    //    {
    //        if (bJoined[iPlane_])
    //            continue;
    //        pPanelSurface_ = panelSurfaces.Element + iPlane_;
    //        if (1.0f - RVLDOTPRODUCT3(pPanelSurface->N, pPanelSurface_->N) > 1e-6)
    //            continue;
    //        e = pPanelSurface_->d - pPanelSurface->d;
    //        if (RVLABS(e) > 1e-6)
    //            continue;
    //        bJoined[iPlane_] = true;
    //    }
    //}
    //surfaces.n = pSurface - surfaces.Element;
    //delete[] bJoined;
    //delete[] panelSurfaces.Element;

    // VN environment model.

    if (model_gt.pVNEnv)
        delete[] model_gt.pVNEnv;
    model_gt.pVNEnv = new VN;
    VN* pVNEnv = model_gt.pVNEnv;
    pVNEnv->CreateEmpty();
    Array2D<float> A;
    A.w = 3;
    A.h = 18;
    A.Element = new float[A.w * A.h];
    CreateConvexTemplate18(A.Element);
    Array<RECOG::PSGM_::Plane> CT;
    CT.n = A.h;
    CT.Element = new RECOG::PSGM_::Plane[CT.n];
    RECOG::PSGM_::CreateTemplate(A, CT);
    float R[9];
    RVLUNITMX3(R);
    float t[3];
    RVLNULL3VECTOR(t);
    Pair<float, float> betaInterval;
    betaInterval.a = 0.0f;
    betaInterval.b = PI;
    Array2D<float> NArray;
    NArray.w = 3;
    NArray.h = 0;
    VNMClusters.n = nPanels;
    RVL_DELETE_ARRAY(VNMClusters.Element);
    VNMClusters.Element = new RECOG::VN_::ModelCluster * [VNMClusters.n];
    for (iPanel = 0; iPanel < nPanels; iPanel++)
    {
        if(bDoor && iPanel == nPanels - 1)
            VNMClusters.Element[iPanel] = pVNEnv->AddModelCluster(iPanel, RVLVN_CLUSTER_TYPE_CONVEX, pose_Arot_A.R, t, 0.5f, CT, betaInterval, NArray, pMem0);
        else
            VNMClusters.Element[iPanel] = pVNEnv->AddModelCluster(iPanel, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.5f, CT, betaInterval, NArray, pMem0);
    }
    pVNEnv->AddOperation(nPanels,     -1, 0, 1, pMem0);
    pVNEnv->AddOperation(nPanels + 1, -1, 2, 3, pMem0);
    pVNEnv->AddOperation(nPanels + 2, -1, nPanels, nPanels + 1, pMem0);
    if (bDoor)
        pVNEnv->AddOperation(nPanels + 3, -1, nPanels + 2, nPanels - 1, pMem0);
    pVNEnv->SetOutput(nPanels + 3);
    pVNEnv->Create(pMem0);
    Array<Vector3<float>> vertices_;
    vertices_.n = 0;
    int iSolid;
    Solid* pSolid;
    int nSolidFaces = 0;
    for (iSolid = 0; iSolid < envSolid.solids.size(); iSolid++)
    {
        pSolid = envSolid.solids[iSolid];
        vertices_.n += pSolid->vertices.n;
        nSolidFaces += pSolid->faces.n;
    }
    vertices_.Element = new Vector3<float>[vertices_.n];
    SolidVertex* pSolidVertex;
    Vector3<float>* pVertex_ = vertices_.Element;
    for (iSolid = 0; iSolid < envSolid.solids.size(); iSolid++)
    {
        pSolid = envSolid.solids[iSolid];        
        for (i = 0; i < pSolid->vertices.n; i++, pVertex_++)
        {
            pSolidVertex = pSolid->vertices.Element + i;
            RVLCOPY3VECTOR(pSolidVertex->P, pVertex_->Element);
        }
    }
    Array<RECOG::VN_::Correspondence5> assoc;
    assoc.Element = new RECOG::VN_::Correspondence5[nPanels * 8];
    RECOG::VN_::Correspondence5* pAssoc = assoc.Element;
    int iVertex = 0;
    for (iPanel = 0; iPanel < nPanels; iPanel++)
    {
        for (j = 0; j < 8; j++, pAssoc++, iVertex++)
        {
            pAssoc->iSPoint = iVertex;
            pAssoc->iMCluster = iPanel;
            pAssoc->iBeta = -1;
        }
    }
    assoc.n = pAssoc - assoc.Element;
    RVL_DELETE_ARRAY(model_gt.d);
    model_gt.d = new float[pVNEnv->featureArray.n];
    pVNEnv->Descriptor(vertices_, assoc, model_gt.d);
    delete[] vertices_.Element;
    delete[] assoc.Element;
    pVNEnv->SetFeatureOffsets(model_gt.d);

    // Visualize VN environment model.

    //pVisualizer = pVisualizationData->pVisualizer;
    //pVNEnv->Display(pVisualizer, 0.02f, model_gt.d);
    //pVisualizer->Run();
    //pVisualizer->Clear();

    /// Vertices.

    vertices.n = vertices_.n;
    RVL_DELETE_ARRAY(vertices.Element);
    vertices.Element = new MOTION::Vertex[vertices.n];
    MOTION::Vertex* pVertex = vertices.Element;
    RVL_DELETE_ARRAY(vertexMem);
    vertexMem = new Pair<int, int>[vertices.n];
    bool* bJoined = new bool[vertices.n];
    memset(bJoined, 0, vertices.n * sizeof(bool));
    vertices.n = 0;
    Pair<int, int>* pVertexIdx = vertexMem;
    iVertex = 0;
    int iSolid_;
    Solid* pSolid_;
    int iVertex_;
    SolidVertex* pSolidVertex_;
    float dP[3];
    int k;
    int l;
    for (iSolid = 0; iSolid < envSolid.solids.size(); iSolid++)
    {
        pSolid = envSolid.solids[iSolid];
        for (i = 0; i < pSolid->vertices.n; i++, iVertex++)
        {
            if (bJoined[iVertex])
                continue;
            bJoined[iVertex] = true;
            pVertex->solidVertices.Element = pVertexIdx;
            pVertexIdx->a = iSolid;
            pVertexIdx->b = i;
            pVertexIdx++;
            pSolidVertex = pSolid->vertices.Element + i;
            iVertex_ = iVertex + 1;
            l = i + 1;
            for (iSolid_ = iSolid; iSolid_ < envSolid.solids.size(); iSolid_++, l = 0)
            {
                pSolid_ = envSolid.solids[iSolid_];
                for (j = l; j < pSolid_->vertices.n; j++, iVertex_++)
                {
                    if (bJoined[iVertex_])
                        continue;
                    pSolidVertex_ = pSolid_->vertices.Element + j;
                    RVLDIF3VECTORS(pSolidVertex_->P, pSolidVertex->P, dP);
                    for (k = 0; k < 3; k++)
                        if (RVLABS(dP[k]) > 1e-6)
                            break;
                    if (k >= 3)
                    {
                        pVertexIdx->a = iSolid_;
                        pVertexIdx->b = j;
                        pVertexIdx++;
                        bJoined[iVertex_] = true;
                    }
                }
            }
            pVertex->solidVertices.n = pVertexIdx - pVertex->solidVertices.Element;
            RVLCOPY3VECTOR(pSolidVertex->P, pVertex->P);
            pVertex++;
        }
    }
    vertices.n = pVertex - vertices.Element;
    delete[] bJoined;

    /// Surfaces.

    RVL_DELETE_ARRAY(surfaces.Element);
    surfaces.Element = new MOTION::PlanarSurface[pVNEnv->featureArray.n];
    MOTION::PlanarSurface* pSurface = surfaces.Element;
    RVL_DELETE_ARRAY(surfaceMem);
    surfaceMem = new uchar[pVNEnv->featureArray.n * sizeof(int) + nSolidFaces * sizeof(Pair<int, int>)];

    // Associate VN features.

    int* pFeatureIdx = (int *)surfaceMem;
    bJoined = new bool[pVNEnv->featureArray.n];
    memset(bJoined, 0, pVNEnv->featureArray.n * sizeof(bool));
    RECOG::VN_::ModelCluster* pVNCluster;
    int iFeature;
    for (iPanel = 0; iPanel < VNMClusters.n; iPanel++, pVNCluster++)
    {
        pVNCluster = VNMClusters.Element[iPanel];
        for (iFeature = pVNCluster->iFeatureInterval.a; iFeature <= pVNCluster->iFeatureInterval.b; iFeature++)
            if(iFeature - pVNCluster->iFeatureInterval.a >= 6)
                bJoined[iFeature] = true;
    }
    float e;
    int iFeature_;
    RECOG::VN_::Feature* pFeature, * pFeature_;
    for (iFeature = 0; iFeature < pVNEnv->featureArray.n; iFeature++)
    {
        if (bJoined[iFeature])
            continue;
        bJoined[iFeature] = true;
        pFeature = pVNEnv->featureArray.Element + iFeature;
        pSurface->VNFeatures.Element = pFeatureIdx;
        *(pFeatureIdx++) = iFeature;
        for (iFeature_ = iFeature + 1; iFeature_ < pVNEnv->featureArray.n; iFeature_++)
        {
            if (bJoined[iFeature_])
                continue;
            pFeature_ = pVNEnv->featureArray.Element + iFeature_;
            if (1.0f - RVLDOTPRODUCT3(pFeature->N, pFeature_->N) > 1e-6)
                continue;
            e = pFeature_->d - pFeature->d;
            if (RVLABS(e) > 1e-6)
                continue;
            *(pFeatureIdx++) = iFeature_;
            bJoined[iFeature_] = true;
        }
        pSurface->VNFeatures.n = pFeatureIdx - pSurface->VNFeatures.Element;
        RVLCOPY3VECTOR(pFeature->N, pSurface->plane.N);
        pSurface->plane.d = pFeature->d;
        pSurface++;
    }
    surfaces.n = pSurface - surfaces.Element;
    delete[] bJoined;

    // Associate solid faces.

    uchar* surfaceMem_ = surfaceMem + pVNEnv->featureArray.n * sizeof(int);
    Pair<int, int>* pSolidFaceIdx = (Pair<int, int> *)surfaceMem_;
    int iFace;
    int iSurface;
    SolidFace* pFace;
    pSurface = surfaces.Element;
    for (iSurface = 0; iSurface < surfaces.n; iSurface++, pSurface++)
    {
        pSurface->solidFaces.Element = pSolidFaceIdx;
        for (iSolid = 0; iSolid < envSolid.solids.size(); iSolid++)
        {
            pSolid = envSolid.solids[iSolid];
            pFace = pSolid->faces.Element;
            for (iFace = 0; iFace < pSolid->faces.n; iFace++, pFace++)
            {
                if (1.0f - RVLDOTPRODUCT3(pFace->N, pSurface->plane.N) < 1e-6)
                {
                    e = pFace->d - pSurface->plane.d;
                    if (RVLABS(e) < 1e-6)
                    {
                        pSolidFaceIdx->a = iSolid;
                        pSolidFaceIdx->b = iFace;
                        pSolidFaceIdx++;
                        if (bDoor)
                            if (iSolid == 4 && iFace == 4)
                                doorRefSurfaceIdx = iSurface;
                    }
                }            
            }
        }
        pSurface->solidFaces.n = pSolidFaceIdx - pSurface->solidFaces.Element;
    }

    ///

    // Create a template for the environment model obtained by a vision system.

    CreateEnvironmentModelTemplete(&model_e, nPanels, CT, betaInterval, &pose_Arot_A);

    // Create a template for the environment model created by a vision system and corrected by touching.

    envSolidx.Copy(&envSolid);
    CreateEnvironmentModelTemplete(&model_x, nPanels, CT, betaInterval, &pose_Arot_A);

    // Allocate memory for computations related to VN models.

    RVL_DELETE_ARRAY(SDFBuff);
    SDFBuff = new float[model_gt.pVNEnv->NodeArray.n];

    //

    delete[] A.Element;
    delete[] CT.Element;
}

void Touch::CreateSimpleTool(
    float a,
    float b,
    float c,
    float d,
    float h,
    float* t)
{
    // Constants.

    //float xy2 = 0.5f * a * a;
    //float xy = sqrt(xy2);
    //float z = (h * h - xy2) / (2.0f * h);
    //float c = b - a;
    float a_ = 0.5f * a;
    float b_ = 0.5f * b;
    float c_ = 0.5f * c;
    float d_ = 0.5f * d;
    float h_ = 0.5f * h;

    // Solid.

    float bboxSize[3];
    RVLSET3VECTOR(bboxSize, a, b, h);
    Pose3D nullPose;
    RVLUNITMX3(nullPose.R);
    RVLNULL3VECTOR(nullPose.t);
    tool.solid.CreateBox(bboxSize, &nullPose);
    float* P;
    P = tool.solid.vertices.Element[4].P;
    RVLSET3VECTOR(P, a_, d_, h_);
    P = tool.solid.vertices.Element[5].P;
    RVLSET3VECTOR(P, a_-c_, d_, h_);
    P = tool.solid.vertices.Element[6].P;
    RVLSET3VECTOR(P, a_-c_, -d_, h_);
    P = tool.solid.vertices.Element[7].P;
    RVLSET3VECTOR(P, a_, -d_, h_);
    if (t)
    {
        SolidVertex* pVertex = tool.solid.vertices.Element;
        for (int iVertex = 0; iVertex < tool.solid.vertices.n; iVertex++, pVertex++)
            RVLSUM3VECTORS(pVertex->P, t, pVertex->P);
    }
    tool.solid.ComputeFaceParams();

    //tool.solid.vertices.n = 6;
    //tool.solid.vertices.Element = new SolidVertex[tool.solid.vertices.n];
    //SolidVertex* pVertex = tool.solid.vertices.Element;
    //RVLSET3VECTOR(pVertex->P, 0.0f, c_, h); pVertex++;
    //RVLSET3VECTOR(pVertex->P, 0.0f, -c_, h); pVertex++;
    //RVLSET3VECTOR(pVertex->P, a_, b_, 0.0f); pVertex++;
    //RVLSET3VECTOR(pVertex->P, -a_, b_, 0.0f); pVertex++;
    //RVLSET3VECTOR(pVertex->P, -a_, -b_, 0.0f); pVertex++;
    //RVLSET3VECTOR(pVertex->P, a_, -b_, 0.0f);
    //int face[5][4] = { {0, 2, 3, -1}, {0, 3, 4, 1}, {1, 4, 5, -1}, {0, 1, 5, 2}, {5, 4, 3, 2} };
    //Array<Array<int>> faces_;
    //faces_.n = 5;
    //faces_.Element = new Array<int>[5];
    //int* facesMem = new int[2 * 3 + 3 * 4];
    //int maxnFaceVertices = 4;
    //int* pFacesMem = facesMem;
    //int iFace;
    //int i;
    //Array<int> *pFace_;
    //for (iFace = 0; iFace < faces_.n; iFace++)
    //{
    //    pFace_ = faces_.Element + iFace;
    //    pFace_->Element = pFacesMem;
    //    for (i = 0; i < maxnFaceVertices; i++)
    //        if (face[iFace][i] >= 0)
    //            *(pFacesMem++) = face[iFace][i];
    //    pFace_->n = pFacesMem - pFace_->Element;
    //}
    //tool.solid.Create(faces_);

    // Visualize solid.

    //Visualizer* pVisualizer = pVisualizationData->pVisualizer;
    //tool.solid.Visualize(pVisualizer);
    //pVisualizer->Run();

    // TCP.

    RVLSET3VECTOR(tool.TCP, 0.0f, 0.0f, h);

    // VN model.

    if (tool.pVN)
        delete[] tool.pVN;
    tool.pVN = new VN;
    tool.pVN->CreateEmpty();
    Array<float> alphaArray;
    alphaArray.n = 0;
    Array<float> betaArray;
    betaArray.n = 0;
    float I[9];
    RVLUNITMX3(I);
    float nullVect[3];
    RVLNULL3VECTOR(nullVect);
    Array2D<float> NArray;
    NArray.w = 3;
    NArray.h = tool.solid.faces.n;
    NArray.Element = new float[NArray.h * NArray.w];
    RVL_DELETE_ARRAY(tool.d);
    tool.d = new float[tool.solid.faces.n];
    float* N = NArray.Element;
    SolidFace* pFace;
    for (int iFace = 0; iFace < tool.solid.faces.n; iFace++, N += 3)
    {
        pFace = tool.solid.faces.Element + iFace;
        RVLCOPY3VECTOR(pFace->N, N);
        tool.d[iFace] = pFace->d;
    }
    tool.pVN->AddModelCluster(0, RVLVN_CLUSTER_TYPE_CONVEX, I, nullVect, 0.5f, alphaArray, betaArray, NArray, pMem0, 0.0f);
    tool.pVN->SetOutput(0);
    tool.pVN->Create(pMem0);
    tool.pVN->SetFeatureOffsets(tool.d);

    // Visualize VN model.

    //Box<float> bbox;
    //P = vertices.Element;
    //InitBoundingBox<float>(&bbox, P);
    //P += 3;
    //for (int i = 1; i < vertices.h; i++, P += 3)
    //    UpdateBoundingBox<float>(&bbox, P);
    //Visualizer* pVisualizer = pVisualizationData->pVisualizer;
    //tool.pVN->Display(pVisualizer, 0.02f, tool.d, NULL, 0.0f, &bbox);
    //pVisualizer->Run();
    //pVisualizer->Clear();

    // Edges.

    //

    delete[] NArray.Element;
}

//#define RVLMOTION_TOUCH_SIMULATION_SAMPLING

void Touch::Simulation()
{
    RVLCOLORS   // For visualization.
    Visualizer* pVisualizer = pVisualizationData->pVisualizer;

    iRndVal = simulationSeed;

    // Parameters.

    // Constants.

    csContactAngleThr = cos(contactAngleThr * DEG2RAD);

    // RGBD-camera intrinsic matirx.

    float K[9];
    IntrinsicCameraMatrix(camera, K);
    float kappazn = kappa * zn;

    // Camera pose w.r.t. the world r.f.

    float az = 0.25f * PI;
    float el = 0.25f * PI;
    float rh = 0.8f;
    float focusPt[3];
    RVLNULL3VECTOR(focusPt);
    pose_C_W.t[0] = rh * cos(az) * cos(el);
    pose_C_W.t[1] = rh * sin(az) * cos(el);
    pose_C_W.t[2] = rh * sin(el);
    float R_W_C[9];
    float* X_C_W = R_W_C; float* Y_C_W = R_W_C + 3; float* Z_C_W = R_W_C + 6;
    RVLDIF3VECTORS(focusPt, pose_C_W.t, Z_C_W);
    float fTmp;
    RVLNORM3(Z_C_W, fTmp);
    RVLSET3VECTOR(X_C_W, Z_C_W[1], -Z_C_W[0], 0.0f);
    RVLNORM3(X_C_W, fTmp);
    RVLCROSSPRODUCT3(Z_C_W, X_C_W, Y_C_W);
    RVLCOPYMX3X3T(R_W_C, pose_C_W.R);
    
    // Camera pose w.r.t. the robot tool r.f.

    Pose3D pose_C_E;
    float tilt = -0.1f;
    float cs = cos(tilt);
    float sn = sin(tilt);
    RVLROTX(cs, sn, pose_C_E.R);
    RVLUNITMX3(pose_C_E.R);
    RVLSET3VECTOR(pose_C_E.t, 0.0f, -0.1f, 0.0f);

    // Tool pose w.r.t. the scene r.f.

    Pose3D pose_W_C;
    RVLINVTRANSF3D(pose_C_W.R, pose_C_W.t, pose_W_C.R, pose_W_C.t);
    RVLCOMPTRANSF3D(pose_C_E.R, pose_C_E.t, pose_W_C.R, pose_W_C.t, pose_W_E.R, pose_W_E.t);
    RVLINVTRANSF3D(pose_W_E.R, pose_W_E.t, pose_E_W.R, pose_E_W.t);

    // TCP w.r.t. the robot tool r.f.

    float TCP_E[3];
    RVLSET3VECTOR(TCP_E, 0.0f, 0.0f, 0.15f);

    // model_gt - true model 
    
    model_gt.camera = camera;
    RVLCOPYMX3X3(K, model_gt.K);
    model_gt.kappa = kappa;
    model_gt.kappazn = kappazn;
    model_gt.pose_C_E = pose_C_E;
    RVLCOPY3VECTOR(TCP_E, model_gt.TCP_E);
    model_gt.pose_A_E = doorPose;

    // Visualize true model transformed in r.f. E.

    //{
    //    Solid* pSolid;
    //    SolidVertex* pSolidVertex;
    //    float V3Tmp[3];
    //    for (int iSolid = 0; iSolid < envSolidx.solids.size(); iSolid++)
    //    {
    //        pSolid = envSolidx.solids[iSolid];
    //        for (int i = 0; i < pSolid->vertices.n; i++)
    //        {
    //            pSolidVertex = pSolid->vertices.Element + i;
    //            RVLTRANSF3(pSolidVertex->P, pose_W_E.R, pose_W_E.t, V3Tmp);
    //            RVLCOPY3VECTOR(V3Tmp, pSolidVertex->P);
    //        }
    //    }
    //    envSolidx.Visualize(pVisualizer, darkGreen);
    //}

    // Error vector.

    float x[RVLMOTION_TOUCH_NUM_PARAMS];
    SimulateVisionWithError(x);

    // Test model correction.

    //UpdateEnvironmentModel(&model_e, x, &model_x);
    //{
    //    int iFeature;
    //    RECOG::VN_::Feature* pFeatureGT, * pFeaturex;
    //    MOTION::Plane plane_W;
    //    RVLINVTRANSF3D(pose_W_E.R, pose_W_E.t, pose_E_W.R, pose_E_W.t);
    //    float eN, ed;
    //    for (iFeature = 0; iFeature < model_gt.pVNEnv->featureArray.n; iFeature++)
    //    {
    //        pFeatureGT = model_gt.pVNEnv->featureArray.Element + iFeature;
    //        pFeaturex = model_x.pVNEnv->featureArray.Element + iFeature;
    //        RVLPLANETRANSF3(pFeaturex->N, pFeaturex->d, pose_E_W.R, pose_E_W.t, plane_W.N, plane_W.d);
    //        RVLDIF3VECTORS(pFeatureGT->N, plane_W.N, V3Tmp);
    //        eN = sqrt(RVLDOTPRODUCT3(V3Tmp, V3Tmp));
    //        ed = pFeatureGT->d - plane_W.d;
    //        printf("eN=%f ed=%f\n", eN, ed);
    //    }
    //}

    float x_gt[RVLMOTION_TOUCH_NUM_PARAMS];
    memcpy(x_gt, x, RVLMOTION_TOUCH_NUM_PARAMS * sizeof(float));
    float E_gt = 0.0f;
    int i;
    for (i = 0; i < RVLMOTION_TOUCH_NUM_PARAMS; i++)
        E_gt += x_gt[i] * x_gt[i] / varx[i];
    E_gt *= alpha;
    printf("E_gt=%f\n", E_gt);
    memset(x, 0, RVLMOTION_TOUCH_NUM_PARAMS * sizeof(float));
    //UpdateEnvironmentModel(&model_e, x, &model_x);
   
    /// Sample points on the surface of the target object reconstructed by the simulated vision system.

    //model_x.pVNEnv->CopyDescriptor(model_x.d);
    //model_gt.pVNEnv->CopyDescriptor(model_gt.d);

    // Parameters.
    
    float rayDist = 0.01f;
    float approachDist = 0.3f;

    //

    Array<MOTION::TouchSample> samples;

#ifdef RVLMOTION_TOUCH_SIMULATION_SAMPLING
    Sample(nSamples, rayDist, samples);
#else
    nSamples = 1;
    samples.n = nSamples;
    samples.Element = new MOTION::TouchSample[1];
    MOTION::TouchSample* pSample = samples.Element;
    Solid* pSolid = envSolidx.solids[3];
    float P_W_[4][3];
    int iRefVertex[] = { 2, 6, 1, 3 };
    float* P_E_;
    for (i = 0; i < 4; i++)
    {
        P_E_ = pSolid->vertices.Element[iRefVertex[i]].P;
        RVLTRANSF3(P_E_, pose_E_W.R, pose_E_W.t, P_W_[i]);
    }
    RVLSUM3VECTORS(P_W_[0], P_W_[1], pSample->P);
    RVLSCALE3VECTOR(pSample->P, 0.5f, pSample->P);
    float V3Tmp[3];
    RVLDIF3VECTORS(P_W_[2], P_W_[0], V3Tmp);
    RVLSCALE3VECTOR(V3Tmp, 0.5f, V3Tmp);
    RVLSUM3VECTORS(pSample->P, V3Tmp, pSample->P);
    RVLDIF3VECTORS(P_W_[0], P_W_[3], V3Tmp);
    RVLNORM3(V3Tmp, fTmp);
    fTmp = 0.015f + 0.005f;
    RVLSCALE3VECTOR(V3Tmp, fTmp, V3Tmp);
    RVLSUM3VECTORS(pSample->P, V3Tmp, pSample->P);
    pSample->iAxis = 0;
    pSample->direction = -1.0f;
#endif

    //Sample(10000, rayDist, samples);
    //{
    //    model_gt.pVNEnv->Display(pVisualizer, 0.02f, model_gt.d);
    //    Array<Point> visPts;
    //    visPts.Element = new Point[samples.n];
    //    visPts.n = samples.n;
    //    for (int iSample = 0; iSample < samples.n; iSample++)
    //    {
    //        RVLCOPY3VECTOR(samples.Element[iSample].P, visPts.Element[iSample].P);
    //    }
    //    pVisualizer->DisplayPointSet<float, Point>(visPts, darkGreen, 4.0f);
    //    pVisualizer->Run();
    //}

    /// Touches.

    // Create tool for touches.

    toolMoved.Copy(&(tool.solid));

    toolMoved.pVisualizer = pVisualizer;

    // envSolid_E <- envSolid transformed to the r.f. E

    envSolid_E.Copy(&envSolid);
    envSolid_E.Move(&envSolid, &pose_W_E);

    // Visualize true environment model transformed to the r.f. E.

    envSolid_E.Visualize(pVisualizer, black);
    //pVisualizer->Run();

    // For each touch: move tool to the sample point or to the contact with an obstacle.

    //Array<Pose3D> touches_W;
    //touches_W.Element = new Pose3D[nSamples];
    //touches_W.n = 0;
    //Pose3D* pTouch_W;
    //float P_W[3], P1_W[3], P2_W[3];
    //Array<Pair<RECOG::VN_::SurfaceRayIntersection, RECOG::VN_::SurfaceRayIntersection>>* pIntersection;
    //Pair<RECOG::VN_::SurfaceRayIntersection, RECOG::VN_::SurfaceRayIntersection>* pIntersectionSegment;
    //int iIntersectionSegment;
    //int iSample;
    //float moveDist;
    //float t;
    //float R_W_G[9];
    //float* X_G_W = R_W_G;
    //float* Y_G_W = R_W_G + 3;
    //float* Z_G_W = R_W_G + 6;
    //float move[3];
    //float P2_E[3];
    //float moveDist_ = 2.0f * approachDist;
    //MOTION::TouchData touch_E;
    //for (iSample = 0; iSample < nSamples; iSample++)
    //{
    //    pSample = samples.Element + iSample;
    //    moveDist = pSample->direction * approachDist;
    //    RVLCOPY3VECTOR(pSample->P, P1_W);
    //    RVLCOPY3VECTOR(pSample->P, P2_W);
    //    P1_W[pSample->iAxis] = pSample->P[pSample->iAxis] - moveDist;
    //    P2_W[pSample->iAxis] = pSample->P[pSample->iAxis] + moveDist;
    //    pTouch_W = touches_W.Element + touches_W.n;
    //    RVLNULLMX3X3(R_W_G);
    //    Z_G_W[pSample->iAxis] = pSample->direction;
    //    X_G_W[(pSample->iAxis + 1) % 3] = 1.0f;
    //    RVLCROSSPRODUCT3(Z_G_W, X_G_W, Y_G_W);
    //    RVLCOPYMX3X3T(R_W_G, pTouch_W->R);
    //    RVLCOPY3VECTOR(P1_W, pTouch_W->t);
    //    touches_W.n++;
    //    RVLCOMPTRANSF3D(pose_W_E.R, pose_W_E.t, pTouch_W->R, pTouch_W->t, touch_E.pose.R, touch_E.pose.t);
    //    toolMoved.Move(&(tool.solid), &(touch_E.pose));
    //    RVLTRANSF3(P2_W, pose_W_E.R, pose_W_E.t, P2_E);
    //    RVLDIF3VECTORS(P2_E, touch_E.pose.t, move);
    //    t = toolMoved.FreeMove(move, &envSolid_E);
    //    fTmp = t / moveDist_;
    //    RVLSCALE3VECTOR(move, fTmp, V);
    //    RVLSUM3VECTORS(touch_E.pose.t, V, touch_E.pose.t);
    //    touch_E.contact.iToolFeature = 0;
    //    touch_E.contact.iEnvFeature.a = 3;
    //    //touch_E.iEnvFeature.b = 12;
    //    //touch_E.type = RVLMOTION_TOUCH_CONTACT_TYPE_EDGE_EDGE;
    //    touch_E.contact.iEnvFeature.b = 2;
    //    touch_E.contact.type = RVLMOTION_TOUCH_CONTACT_TYPE_POINT_PLANE;
    //    touches_E.push_back(touch_E);

    //    // Touch visualization.

    //    toolMoved.Move(&(tool.solid), &(touch_E.pose));
    //    toolMoved.Visualize(pVisualizer, yellow);
    //    {
    //        Array<Point> visPts;
    //        Point visPtMem[3];
    //        visPts.Element = visPtMem;
    //        visPts.n = 1;
    //        RVLCOPY3VECTOR(toolMoved.vertices.Element[0].P, visPts.Element[0].P);
    //        pVisualizer->DisplayPointSet<float, Point>(visPts, yellow, 6.0f);
    //        pSolid = envSolidx.solids[3];
    //        SolidEdge* pEdge = pSolid->edges.Element + 12;
    //        SolidEdge* pEdge_ = pSolid->edges.Element + pEdge->iNext;
    //        RVLCOPY3VECTOR(pSolid->vertices.Element[pEdge->iVertex].P, visPts.Element[0].P);
    //        RVLCOPY3VECTOR(pSolid->vertices.Element[pEdge_->iVertex].P, visPts.Element[1].P);
    //        RVLCOPY3VECTOR(pSolid->vertices.Element[0].P, visPts.Element[2].P);
    //        //visPts.n = 3;
    //        visPts.n = 2;
    //        pVisualizer->DisplayPointSet<float, Point>(visPts, red, 6.0f);
    //    }
    //    //pVisualizer->Run();
    //}

    ///

    // Visualization of samples and touches.

    //{
    //    model_gt.pVNEnv->Display(pVisualizer, 0.02f, model_gt.d);
    //    float visVectSize = 0.03f;
    //    Array<Point> visPts;
    //    visPts.Element = new Point[2 * samples.n];
    //    visPts.n = samples.n;
    //    Array<Pair<int, int>> visLines;
    //    visLines.Element = new Pair<int, int>[samples.n];
    //    visLines.n = samples.n;
    //    int iSample_;
    //    for (iSample = 0; iSample < samples.n; iSample++)
    //    {
    //        iSample_ = iSample + samples.n;
    //        RVLCOPY3VECTOR(samples.Element[iSample].P, visPts.Element[iSample].P);            
    //        RVLCOPY3VECTOR(samples.Element[iSample].P, visPts.Element[iSample_].P);
    //        pSample = samples.Element + iSample;
    //        visPts.Element[iSample_].P[pSample->iAxis] -= pSample->direction * visVectSize;
    //        visLines.Element[iSample].a = iSample;
    //        visLines.Element[iSample].b = iSample_;
    //    }
    //    pVisualizer->DisplayPointSet<float, Point>(visPts, darkGreen, 4.0f);
    //    visPts.n = 2 * samples.n;
    //    pVisualizer->DisplayLines(visPts, visLines, red);
    //    visPts.n = touches_W.n;
    //    int iTouch;
    //    for (iTouch = 0; iTouch < touches_W.n; iTouch++)
    //    {
    //        RVLCOPY3VECTOR(touches_W.Element[iTouch].t, visPts.Element[iTouch].P);
    //    }
    //    pVisualizer->DisplayPointSet<float, Point>(visPts, blue, 4.0f);
    //    pVisualizer->Run();
    //}

    // Check distances between the true model and the estimated model.

    //int iActiveFeature;
    //int iTouch;
    //Pose3D* pTouch;
    //float e;
    //float emin;
    //float emax;
    //float eavg;
    //float P_E[3];
    //for (iTouch = 0; iTouch < touches_W.n; iTouch++)
    //{
    //    pTouch = touches_W.Element + iTouch;
    //    RVLTRANSF3(pTouch->t, pose_W_E.R, pose_W_E.t, P_E);
    //    e = model_x.pVNEnv->Evaluate(P_E, SDFBuff, iActiveFeature, true, model_x.d);
    //    if (iTouch == 0)
    //        emin = emax = eavg = e;
    //    else
    //    {
    //        if (e < emin)
    //            emin = e;
    //        else if (e > emax)
    //            emax = e;
    //        eavg += e;
    //    }
    //}
    //eavg /= (float)(touches_W.n);
    //printf("emin=%f emax=%f eavg=%f\n", emin, emax, eavg);

#ifdef RVLMOTION_TOUCH_SIMULATION_SAMPLING
    delete[] samples.Element;
#endif

    /// Find the optimal model. 

    // Transform touches to RF E.
    // Associate touches with surfaces.

    //int iTouch;
    //float Z_Ek_E[3];
    //float e;
    //float mine;
    //RECOG::VN_::Feature* pFeature;
    //for (iTouch = 0; iTouch < touches_W.n; iTouch++)
    //{
    //    pTouch_W = touches_W.Element + iTouch;
    //    pTouch_E = touches_E.Element + iTouch;
    //    RVLCOMPTRANSF3D(pose_W_E.R, pose_W_E.t, pTouch_W->R, pTouch_W->t, pTouch_E->pose.R, pTouch_E->pose.t);
    //    RVLCOPYCOLMX3X3(pTouch_E->pose.R, 2, Z_Ek_E);
    //    mine = -1.0f;
    //    pTouch_E->iEnvFeature.a = -1;
    //    for (iSurface = 0; iSurface < surfaces.n; iSurface++)
    //    {
    //        pSurface = surfaces.Element + iSurface;
    //        pFeature = model_e.pVNEnv->featureArray.Element + pSurface->VNFeatures.Element[0];
    //        if (RVLDOTPRODUCT3(Z_Ek_E, pFeature->N) > -COS45)
    //            continue;
    //        e = RVLDOTPRODUCT3(pFeature->N, pTouch_E->pose.t) - pFeature->d;
    //        e = RVLABS(e);
    //        if (mine < 0.0f || e < mine)
    //        {
    //            mine = e;
    //            pTouch_E->iEnvFeature.a = iSurface;
    //        }
    //    }

    //    // Visualize touch.

    //    toolMoved.Move(&(tool.solid), &(pTouch_E->pose));
    //    std::vector<vtkSmartPointer<vtkActor>> toolActors = toolMoved.Visualize(pVisualizer, yellow);
    //    pVisualizer->Run();
    //    pVisualizer->Clear(toolActors);
    //}

    // Line-convex intersection test.

    //{
    //    RVLNULL3VECTOR(P_W_[0]);
    //    RVLSET3VECTOR(P_W_[1], 0.0f, -0.5f, 0.0f);
    //    pSolid = envSolid.solids[3];
    //    float t_[2];
    //    pSolid->Intersect(P_W_[0], P_W_[1], t_);
    //    RVLNULL3VECTOR(P_W_[2]);
    //    P_W_[2][1] -= t_[0];
    //    RVLNULL3VECTOR(P_W_[3]);
    //    P_W_[3][1] -= t_[1];
    //    float P_E__[4][3];
    //    for (i = 0; i < 4; i++)
    //        RVLTRANSF3(P_W_[i], pose_W_E.R, pose_W_E.t, P_E__[i]);
    //    Array<Point> visPts;
    //    visPts.n = 4;
    //    Point visPtMem[4];
    //    visPts.Element = visPtMem;
    //    for (i = 0; i < 4; i++)
    //        RVLCOPY3VECTOR(P_E__[i], visPts.Element[i].P);
    //    Array<Pair<int, int>> visLines;
    //    visLines.n = 2;
    //    Pair<int, int> visLineMem[2];
    //    visLines.Element = visLineMem;
    //    visLines.Element[0].a = 0;
    //    visLines.Element[0].b = 2;
    //    visLines.Element[1].a = 3;
    //    visLines.Element[1].b = 1;
    //    pVisualizer->DisplayLines(visPts, visLines, magenta);
    //}
    //pVisualizer->Run();

    /// Touch and correct.

    if (bRefPtConstraints)
        RefPts();

    Box<float> bbox;
    SceneBBox(&model_e, &bbox);
    std::vector<MOTION::TouchData> touches_E;
    Pose3D pose_Ek_E;
    Array<Vector3<float>> path;
    path.Element = new Vector3<float>[2];
    MOTION::TouchPoint touchPt;
    float xOpt[RVLMOTION_TOUCH_NUM_PARAMS];
    Array<MOTION::TouchData> touches;
    std::vector<MOTION::Contact> contacts;
    MOTION::TouchData touch;
    int iLastSegment;
    float PTouch[3];
    float V[3];
    while(touches_E.size() < nSimulationTouches)
    {
        // Touch.

        touchPt.iPanel = touchPt.iFace = -1;
        //if (touches_E.size() == 0)
        //    touchPt.iPanel = 3;
        RndTouchPoint(touchPt);
        do
        {
            PlanTouch(touchPt, pose_Ek_E, path, PTouch);
            {
                Point visPt;
                RVLCOPY3VECTOR(PTouch, visPt.P);
                Array<Point> visPts;
                visPts.n = 1;
                visPts.Element = &visPt;
                pVisualizer->DisplayPointSet<float, Point>(visPts, red, 6.0f);
                pVisualizationData->pVisualizer->Clear(pVisualizationData->envActors);
                pVisualizationData->envActors = envSolidx.Visualize(pVisualizationData->pVisualizer, green);
            }
            //{
            //    toolMoved.Move(&(tool.solid), &pose_Ek_E);
            //    toolMoved.Visualize(pVisualizer, white);
            //    Pose3D pose_Ek_E_ = pose_Ek_E;
            //    RVLCOPY3VECTOR(path.Element[0].Element, pose_Ek_E_.t);
            //    toolMoved.Move(&(tool.solid), &pose_Ek_E_);
            //    toolMoved.Visualize(pVisualizer, white);
            //    RVLCOPY3VECTOR(path.Element[1].Element, pose_Ek_E_.t);
            //    toolMoved.Move(&(tool.solid), &pose_Ek_E_);
            //    toolMoved.Visualize(pVisualizer, white);
            //}
            SimulateMove(pose_Ek_E, path, pose_Ek_E, V, iLastSegment);
            if (iLastSegment == 2)
            {
                printf("No touch.\n");
                break;
            }
            toolMoved.Move(&(tool.solid), &pose_Ek_E);
            toolMoved.Visualize(pVisualizer, yellow);
            pVisualizer->Run();
            touch.pose = pose_Ek_E;
            RVLCOPY3VECTOR(V, touch.V);
            touch.iFirstContact = -1;
            touches_E.push_back(touch);

            // Correct camera-robot model using information obtained by touches.

            touches.n = touches_E.size();
            touches.Element = touches_E.data();
            if (!Correction(x, touches, contacts, xOpt))
                break;

            // Visualize the corrected model.

            UpdateEnvironmentModel(&model_e, xOpt, &model_x);
            UpdateDoorOrientation(&model_x);
            UpdateEnvironmentVNModel(&model_e, xOpt, &model_x);
            pVisualizationData->pVisualizer->Clear(pVisualizationData->envActors);
            pVisualizationData->envActors = envSolidx.Visualize(pVisualizationData->pVisualizer, green);
            pVisualizationData->envActors.push_back(pVisualizer->DisplayReferenceFrame(&(model_x.pose_A_E), 0.2f));
            //pVisualizationData->envActors.push_back(model_x.pVNEnv->Display(pVisualizationData->pVisualizer, 0.01f, NULL, NULL, 0.0f, &bbox));
            pVisualizationData->pVisualizer->Run();

            //

            memcpy(x, xOpt, RVLMOTION_TOUCH_NUM_PARAMS * sizeof(float));
        //} while (iLastSegment == 0);
        } while (false);
    }
    delete[] path.Element;

    //Sample(10000, 0.1f, samples);
    //{
    //    model_gt.pVNEnv->Display(pVisualizer, 0.02f, model_gt.d);
    //    Array<Point> visPts;
    //    visPts.Element = new Point[samples.n];
    //    visPts.n = samples.n;
    //    for (iSample = 0; iSample < samples.n; iSample++)
    //    {
    //        RVLCOPY3VECTOR(samples.Element[iSample].P, visPts.Element[iSample].P);
    //    }
    //    pVisualizer->DisplayPointSet<float, Point>(visPts, darkGreen, 4.0f);
    //    pVisualizer->Run();
    //}

    ///

    //delete[] plane_E;
    //delete[] touches_W.Element;
    delete[] samples.Element;
}

void Touch::SimulateVisionWithError(float* x)
{
    float* l = x;
    float* pgz = l + 4;
    float* phz = pgz + 1;
    float* phi = phz + 1;
    float* s = phi + 3;
    float* c = s + 3;

    // Simulated error.

    //Array<Point> visPts;
    //visPts.Element = new Point[1000];
    //Point* pVisPt = visPts.Element;
    //for (i = 0; i < 1000; i++, pVisPt++)
    //    PseudoRndSampleUnitSphere(pVisPt->P, rndVal, iRndVal);
    //visPts.n = pVisPt - visPts.Element;
    //pVisualizer->DisplayPointSet<float, Point>(visPts, green, 6.0f);
    //delete[] visPts.Element;
    //pVisualizer->Run();
    //pVisualizer->Clear();

    int i;
    for (i = 0; i < 4; i++)
        l[i] = stdl[i] * GaussPseudoRandBM<float>(rndVal, iRndVal);
    *pgz = stdgz * GaussPseudoRandBM<float>(rndVal, iRndVal);
    float gz = *pgz;
    *phz = stdhz * GaussPseudoRandBM<float>(rndVal, iRndVal);
    float hz = *phz;
    for (i = 0; i < 3; i++)
    {
        s[i] = stds * GaussPseudoRandBM<float>(rndVal, iRndVal);
        c[i] = stdc * GaussPseudoRandBM<float>(rndVal, iRndVal);
    }
    float U[3];
    PseudoRndSampleUnitSphere(U, rndVal, iRndVal);
    int iTmp;
    float ph = stdphirad * GaussPseudoRandBM<float>(rndVal, iRndVal);
    RVLSCALE3VECTOR(U, ph, phi);

    // RGBD-camera with calibration error.

    Camera eCamera;
    eCamera.fu = camera.fu - l[0];
    eCamera.fv = camera.fv - l[1];
    eCamera.uc = camera.uc - l[2];
    eCamera.vc = camera.vc - l[3];
    float Ke[9];
    IntrinsicCameraMatrix(eCamera, Ke);
    float kappae = kappa - gz;
    float kappazne = kappa * zn - hz;
    float zne = kappazne / kappae;

    // Camera extrinsics with calibration error.

    ph = sqrt(RVLDOTPRODUCT3(phi, phi));
    RVLSCALE3VECTOR2(phi, ph, U);
    float Re[9];
    AngleAxisToRot<float>(U, ph, Re);
    float invRe[9];
    RVLCOPYMX3X3T(Re, invRe);
    Pose3D pose_C_E = model_gt.pose_C_E;
    Pose3D pose_C_E_e;
    RVLMXMUL3X3(pose_C_E.R, invRe, pose_C_E_e.R);
    RVLDIF3VECTORS(pose_C_E.t, s, pose_C_E_e.t);

    // TCP with calibration error.

    float TCP_E_e[3];
    RVLDIF3VECTORS(model_gt.TCP_E, c, TCP_E_e);

    // model_e - model obtained by calibration with error.

    model_e.camera = eCamera;
    RVLCOPYMX3X3(Ke, model_e.K);
    model_e.kappa = kappae;
    model_e.kappazn = kappazne;
    model_e.pose_C_E = pose_C_E_e;
    RVLCOPY3VECTOR(TCP_E_e, model_e.TCP_E);

    //// Plane measurement by vision with calibration error.

    float a[3];
    float b;
    float a_[3];
    float b_;
    float D[9];
    float invD[9];
    float M[9], V[3];
    AuxParams(&model_e, &model_gt, gz, hz, a, b, a_, b_, D, invD, M, V);

    //// a <- gz / (kappae * zne) * Z_E_C.T

    //float *Z_E_C = pose_C_E.R + 6;
    //fTmp = gz / kappazne;
    //RVLSCALE3VECTOR(Z_E_C, fTmp, a);

    //// b <- a * (t_C_E + s) + 1 + hz / (kappae * zne)
    //
    // b = RVLDOTPRODUCT3(a, pose_C_E.t) + 1.0f + hz / kappazne;

    //// a_ <- gz / (kappa * zn) * Z_C_E_e.T

    //float Z_C_E_e[3];
    //RVLCOPYCOLMX3X3(pose_C_E_e.R, 2, Z_C_E_e);
    //fTmp = gz / kappazn;
    //RVLSCALE3VECTOR(Z_C_E_e, fTmp, a_);

    //// b_ <- -a_ * t_E_C_e + kappazne / kappazn

    // b_ = -RVLDOTPRODUCT3(a_, pose_C_E_e.t) + kappazne / kappazn;

    //// D <- R_C_E * K.inv() * Ke * R_C_E_e.T

    ////cv::Mat cvD(3, 3, CV_32FC1);
    ////float *D = (float*)(cvD.data);
    //float M3x3Tmp1[9];
    //RVLMXMUL3X3T2(Ke, pose_C_E_e.R, M3x3Tmp1);
    //float invK[9];
    //InvIntrinsicCameraMatrix(K, invK);
    //float M3x3Tmp2[9];
    //RVLMXMUL3X3(pose_C_E.R, invK, M3x3Tmp2);
    //RVLMXMUL3X3(M3x3Tmp2, M3x3Tmp1, D);

    //// invD <- R_C_E_e * Ke.inv() * K * R_C_E.T

    //RVLMXMUL3X3T2(K, pose_C_E.R, M3x3Tmp1);
    //float invKe[9];
    //InvIntrinsicCameraMatrix(Ke, invKe);
    //RVLMXMUL3X3(pose_C_E_e.R, invKe, M3x3Tmp2);
    //RVLMXMUL3X3(M3x3Tmp2, M3x3Tmp1, invD);

    /// Compute planes estimated by the vision system from the true planes.

    int iSurface;
    float V3Tmp[3];
    float r;
    MOTION::PlanarSurface* pSurface = surfaces.Element;
    MOTION::Plane Plane_E;
    MOTION::Plane* pPlane_e = model_e.plane;
    for (iSurface = 0; iSurface < surfaces.n; iSurface++, pSurface++, pPlane_e++)
    {
        // Plane_E <- surfaces.Element[iSurface].plane transformed to r.f. E 

        RVLPLANETRANSF3(pSurface->plane.N, pSurface->plane.d, pose_W_E.R, pose_W_E.t, Plane_E.N, Plane_E.d);

        VisionPlane(&Plane_E, a_, b_, D, &pose_C_E, &pose_C_E_e, pPlane_e);

        //// r <- Plane_E.d - Plane_E.N.T * pose_C_E.t 

        //r = Plane_E.d - RVLDOTPRODUCT3(Plane_E.N, pose_C_E.t);

        //// model_e.plane[iSurface].N <- (D.T * Plane_E.N - r * a_.T).norm

        //RVLMULMX3X3TVECT(D, Plane_E.N, V3Tmp);
        //RVLSCALE3VECTOR(a_, r, pPlane_e->N);
        //RVLDIF3VECTORS(V3Tmp, pPlane_e->N, pPlane_e->N);
        //RVLNORM3(pPlane_e->N, fTmp);

        //// model_e.plane[iSurface].d <- (b_ * r  + (Plane_E.N.T * D * t_C_E_e) / norm(D.T * Plane_E.N - r * a_.T)

        //pPlane_e->d = (r * b_ + RVLDOTPRODUCT3(V3Tmp, pose_C_E_e.t)) / fTmp;

        // printf("%f %f %f %f\n", pPlane_e->N[0], pPlane_e->N[1], pPlane_e->N[2], pPlane_e->d);
    }

    /// Compute vertices estimated by the vision system from the true vertices.

    int iVertex;
    MOTION::Vertex* pVertex = vertices.Element;
    Vector3<float>* pVertex_e = model_e.vertex;
    for (iVertex = 0; iVertex < vertices.n; iVertex++, pVertex++, pVertex_e++)
        VisionVertex(pVertex->P, a, b, invD, pose_C_E, pose_C_E_e, pVertex_e->Element);

    /// Compute door parameters estimated by the vision system from the true door parameters.

    if (bDoor)
    {
        VisionVertex(model_gt.pose_A_E.t, a, b, invD, pose_C_E, pose_C_E_e, model_e.pose_A_E.t);
        float P_W[3];
        RVLCOPYCOLMX3X3(model_gt.pose_A_E.R, 2, P_W);
        RVLSUM3VECTORS(model_gt.pose_A_E.t, P_W, P_W);
        VisionVertex(P_W, a, b, invD, pose_C_E, pose_C_E_e, model_e.PAxis_E);
        UpdateDoorOrientation(&model_e);
    }

    ///

    UpdateVerticesAndPlanes(&model_e);

    // Test vertex-plane consistency.

    //TestVertexPlaneConsistency();

    ////

    // Visualization of the envornoment model reconstructed with error.

    RVLCOLORS
    Visualizer* pVisualizer = pVisualizationData->pVisualizer;
    pVisualizationData->envActors = envSolidx.Visualize(pVisualizer, darkGreen);
    pVisualizationData->envActors.push_back(pVisualizer->DisplayReferenceFrame(&(model_e.pose_A_E), 0.2f));

    // Environment VN model reconstructed with error.

    int iFeature;
    RECOG::VN_::Feature* pFeatureSrc = model_gt.pVNEnv->featureArray.Element;
    RECOG::VN_::Feature* pFeatureTgt = model_e.pVNEnv->featureArray.Element;
    MOTION::Plane Plane_E_e;
    for (iFeature = 0; iFeature < model_e.pVNEnv->featureArray.n; iFeature++, pFeatureSrc++, pFeatureTgt++)
    {
        RVLPLANETRANSF3(pFeatureSrc->N, pFeatureSrc->d, pose_W_E.R, pose_W_E.t, Plane_E.N, Plane_E.d);
        VisionPlane(&Plane_E, a_, b_, D, &pose_C_E, &pose_C_E_e, &Plane_E_e);
        RVLCOPY3VECTOR(Plane_E_e.N, pFeatureTgt->N);
        pFeatureTgt->d = Plane_E_e.d;
    }

    // Visualize environment VN model.

    Box<float> bbox;
    SceneBBox(&model_e, &bbox);
    //model_e.pVNEnv->Display(pVisualizer, 0.01f, NULL, NULL, 0.0f, &bbox);
    //pVisualizer->Run();

    // Test mapping true planes to the planes reconstructed usint a vision system with error.

    //TestPlaneMapping(&model_e);
}

void Touch::Correct(
    MOTION::TouchModel* pModelSrc,
    float *x,
    MOTION::TouchModel* pModelTgt)
{
    float* l = x;
    float gz = x[4];
    float hz = x[5];
    float* phi = x + 6;
    float* s = phi + 3;
    float* c = s + 3;

    // pModelTgt->K <- pModelSrc->K + L

    pModelTgt->camera.fu = pModelSrc->camera.fu + l[0];
    pModelTgt->camera.fv = pModelSrc->camera.fv + l[1];
    pModelTgt->camera.uc = pModelSrc->camera.uc + l[2];
    pModelTgt->camera.vc = pModelSrc->camera.vc + l[3];
    IntrinsicCameraMatrix(pModelTgt->camera, pModelTgt->K);

    // pModelTgt->kappa <- pModelSrc->kappa + gz

    pModelTgt->kappa = pModelSrc->kappa + gz;

    // pModelTgt->kappazn <- pModelSrc->kappazn + hz

    pModelTgt->kappazn = pModelSrc->kappazn + hz;

    // pModelTgt->pose_C_E.R <- pModelSrc->pose_C_E.R * R(phi)

    float ph = sqrt(RVLDOTPRODUCT3(phi, phi));
    if (ph > 1e-6)
    {
        float U[3];
        RVLSCALE3VECTOR2(phi, ph, U);
        float Rx[9];
        AngleAxisToRot<float>(U, ph, Rx);
        float R_C_E_x[9];
        RVLMXMUL3X3(pModelSrc->pose_C_E.R, Rx, R_C_E_x);
        RVLCOPYMX3X3(R_C_E_x, pModelTgt->pose_C_E.R);
    }
    else
        RVLCOPYMX3X3(pModelSrc->pose_C_E.R, pModelTgt->pose_C_E.R)

    // pModelTgt->pose_C_E.t <- pModelSrc->pose_C_E.t + s

    RVLSUM3VECTORS(pModelSrc->pose_C_E.t, s, pModelTgt->pose_C_E.t);

    // pModelTgt->TCP_E <- pModelSrc->TCP_E + c

    RVLSUM3VECTORS(pModelSrc->TCP_E, c, pModelTgt->TCP_E);
}

void Touch::CorrectVertex(
    float* P0,
    float* a_,
    float b_,
    float *D,
    Pose3D *pPose_C_E_0,
    Pose3D* pPose_C_E_x,
    float *Px)
{
    // pModelx->vertex[iVertex] <- 1.0 / (a_ * pModel0->vertex[iVertex] + b_) * D * (pModel0->vertex[iVertex] - pModel0->pose_C_E.t) + pModelx->pose_C_E.t

    float V3Tmp[3];
    RVLDIF3VECTORS(P0, pPose_C_E_0->t, V3Tmp);
    RVLMULMX3X3VECT(D, V3Tmp, Px);
    float fTmp = RVLDOTPRODUCT3(a_, P0) + b_;
    RVLSCALE3VECTOR2(Px, fTmp, Px);
    RVLSUM3VECTORS(Px, pPose_C_E_x->t, Px);
}

void Touch::CorrectPlane(
    MOTION::Plane* pPlaneSrc,
    float *a,
    float b,
    float *M,
    float *V,
    MOTION::Plane* pPlaneTgt)
{
    // pPlaneTgt->N <- (M.T * pPlaneSrc->N + pPlaneSrc->d * a.T).norm

    RVLMULMX3X3TVECT(M, pPlaneSrc->N, pPlaneTgt->N);
    float V3Tmp[3];
    RVLSCALE3VECTOR(a, pPlaneSrc->d, V3Tmp);
    RVLSUM3VECTORS(pPlaneTgt->N, V3Tmp, pPlaneTgt->N);
    float fTmp = sqrt(RVLDOTPRODUCT3(pPlaneTgt->N, pPlaneTgt->N));
    RVLSCALE3VECTOR2(pPlaneTgt->N, fTmp, pPlaneTgt->N);

    // pPlaneTgt->d <- (b * pPlaneSrc->d + pPlaneSrc->N.T * V) / norm(M * pPlaneSrc->N + pPlaneSrc->d * a.T)

    pPlaneTgt->d = (b * pPlaneSrc->d + RVLDOTPRODUCT3(pPlaneSrc->N, V)) / fTmp;
}

void Touch::Contacts(
    MOTION::TouchData* pTouch_E,
    std::vector<MOTION::Contact>& contacts,
    bool bVisualization)
{
    RVLCOLORS

    pTouch_E->iFirstContact = contacts.size();

    MOTION::Contact contact;
    SolidVertex* pSolidVertex;
    SolidEdge* pEdge, * pToolEdge, * pEnvEdge;
    SolidFace* pSolidFace;
    int iSolid, iFace, iToolEdge, iEnvEdge;
    Pair<float, float> err;

    toolMoved.Move(&(tool.solid), &(pTouch_E->pose));

    if (bVisualization)
        pVisualizationData->robotActors.push_back(toolMoved.Visualize(pVisualizationData->pVisualizer, yellow)[0]);

    // Tool vertex - environment plane contacts.

    contact.type = RVLMOTION_TOUCH_CONTACT_TYPE_POINT_PLANE;
    pSolidVertex = toolMoved.vertices.Element;
    int iContactFace;
    Solid* pSolid;
    SolidFace* contactFacePtr[4];
    int iVertexFace[3];
    int iEdge, iEdge0;
    bool* bFaceToFace = new bool[toolMoved.faces.n];
    memset(bFaceToFace, 0, toolMoved.faces.n * sizeof(bool));
    int i;
    for (int iVertex = 0; iVertex < toolMoved.vertices.n; iVertex++, pSolidVertex++)
    {
        iContactFace = 0;
        iEdge = iEdge0 = pSolidVertex->iEdge;
        do
        {
            pEdge = toolMoved.edges.Element + iEdge;
            contactFacePtr[iContactFace] = toolMoved.faces.Element + pEdge->iFace;
            iVertexFace[iContactFace] = pEdge->iFace;
            iContactFace++;
            iEdge = toolMoved.edges.Element[pEdge->iPrev].iTwin;
        } while (iEdge != iEdge0 && iContactFace < 3);
        for (iSolid = 0; iSolid < envSolidx.solids.size(); iSolid++)
        {
            pSolid = envSolidx.solids[iSolid];
            pSolidFace = pSolid->faces.Element;
            for (iFace = 0; iFace < pSolid->faces.n; iFace++, pSolidFace++)
            {
                if (RVLDOTPRODUCT3(pSolidFace->N, pTouch_E->V) > -csContactAngleThr)
                //if (RVLDOTPRODUCT3(pSolidFace->N, pTouch_E->V) > -1e-6)
                    continue;
                contact.iToolFeature = iVertex;
                contact.iEnvFeature.a = iSolid;
                contact.iEnvFeature.b = iFace;
                pTouch_E->contact = contact;
                err = Error(pTouch_E, true);
                if (RVLABS(err.a) > maxReconstructionError || err.b > maxReconstructionError)
                    continue;
                for (i = 0; i < 3; i++)
                    if (RVLDOTPRODUCT3(contactFacePtr[i]->N, pSolidFace->N) + 1.0f < 1e-6)
                        break;
                if (i < 3)
                {
                    if (bFaceToFace[iVertexFace[i]])
                        continue;
                    bFaceToFace[iVertexFace[i]] = true;
                }
                else
                {
                    contactFacePtr[3] = pSolidFace;
                    if (Intersection(contactFacePtr[0]->N, contactFacePtr[1]->N, contactFacePtr[2]->N, contactFacePtr[3]->N))
                        continue;
                }
                contacts.push_back(contact);
            }
        }
    }
    delete[] bFaceToFace;

    // Edge - edge contacts.

    contact.type = RVLMOTION_TOUCH_CONTACT_TYPE_EDGE_EDGE;
    pToolEdge = toolMoved.edges.Element;
    for (iToolEdge = 0; iToolEdge < toolMoved.edges.n; iToolEdge++, pToolEdge++)
    {
        if (pToolEdge->iVertex > toolMoved.edges.Element[pToolEdge->iNext].iVertex)
            continue;
        contactFacePtr[0] = toolMoved.faces.Element + pToolEdge->iFace;
        contactFacePtr[1] = toolMoved.faces.Element + toolMoved.edges.Element[pToolEdge->iTwin].iFace;
        for (iSolid = 0; iSolid < envSolidx.solids.size(); iSolid++)
        {
            pSolid = envSolidx.solids[iSolid];
            pEnvEdge = pSolid->edges.Element;
            for (iEnvEdge = 0; iEnvEdge < pSolid->edges.n; iEnvEdge++, pEnvEdge++)
            {
                //if (iToolEdge == 8 && iSolid == 3 && iEnvEdge == 12)
                //    int debug = 0;
                if (pEnvEdge->iVertex > pSolid->edges.Element[pEnvEdge->iNext].iVertex)
                    continue;
                contact.iToolFeature = iToolEdge;
                contact.iEnvFeature.a = iSolid;
                contact.iEnvFeature.b = iEnvEdge;
                pTouch_E->contact = contact;
                err = Error(pTouch_E, true);
                if (RVLABS(err.a) > maxReconstructionError || err.b > maxReconstructionError)
                    continue;
                contactFacePtr[2] = pSolid->faces.Element + pEnvEdge->iFace;
                contactFacePtr[3] = pSolid->faces.Element + pSolid->edges.Element[pEnvEdge->iTwin].iFace;
                if (Intersection(contactFacePtr[0]->N, contactFacePtr[1]->N, contactFacePtr[2]->N, contactFacePtr[3]->N))
                    continue;
                contacts.push_back(contact);
            }
        }
    }

    pTouch_E->nContacts = contacts.size() - pTouch_E->iFirstContact;
}

bool Touch::Correction(
    float* xInit,
    Array<MOTION::TouchData> touches_E,
    std::vector<MOTION::Contact> &contacts,
    float* xOpt)
{
    //if (touches_E.n == 5)
    //    int debug = 0;

    // Contacts.

    bool bVisualizeContacts = true;
    //bool bVisualizeContacts = false;
    std::vector<Point> visContactPts;
    std::vector<Pair<int, int>> visContactLines;
    bool bVisualizeOptimization = pVisualizationData->bOptimization;
    SetVisualizeOptimization(false);

    int iTouch;
    MOTION::TouchData* pTouch_E = touches_E.Element;
    for (iTouch = 0; iTouch < touches_E.n; iTouch++, pTouch_E++)
        if(pTouch_E->iFirstContact < 0)
            Contacts(pTouch_E, contacts, bVisualizeContacts);

    if (touches_E.Element[touches_E.n - 1].nContacts == 0)
        return false;

    if (bVisualizeContacts)
    {
        SetVisualizeOptimization(true);
        pTouch_E = touches_E.Element;
        int i;
        for (iTouch = 0; iTouch < touches_E.n; iTouch++, pTouch_E++)
            for (i = 0; i < pTouch_E->nContacts; i++)
            {
                pTouch_E->contact = contacts[pTouch_E->iFirstContact + i];
                Error(pTouch_E);
            }
        pVisualizationData->pVisualizer->Run();
        pVisualizationData->pVisualizer->Clear(pVisualizationData->robotActors);
    }
    pVisualizationData->bOptimization = bVisualizeOptimization;

    // Contact combinations.

    Array<int> nTouchContacts;
    nTouchContacts.n = touches_E.n;
    nTouchContacts.Element = new int[nTouchContacts.n];
    pTouch_E = touches_E.Element;
    for (iTouch = 0; iTouch < touches_E.n; iTouch++, pTouch_E++)
        nTouchContacts.Element[iTouch] = (pTouch_E->nContacts > 0 ? pTouch_E->nContacts : 1);
    Array2D<int> touchContactCombinations;
    Combinations(nTouchContacts, touchContactCombinations);
    delete[] nTouchContacts.Element;

    // Optimization.

    printf("Optimization...\n");
    float x[RVLMOTION_TOUCH_NUM_PARAMS];
    memcpy(x, xInit, RVLMOTION_TOUCH_NUM_PARAMS * sizeof(float));
    int iTouchContactCombination;
    int* touchContacts_;
    MOTION::TouchLMError E, minE;
    minE.E = -1.0f;
    int *optimalContacts = new int[touches_E.n];     // Only for debugging prupose!!!
    for (iTouchContactCombination = 0; iTouchContactCombination < touchContactCombinations.h; iTouchContactCombination++)
    {
        touchContacts_ = touchContactCombinations.Element + iTouchContactCombination * touchContactCombinations.w;
        pTouch_E = touches_E.Element;
        for (iTouch = 0; iTouch < touches_E.n; iTouch++, pTouch_E++)
            if(pTouch_E->nContacts > 0)
                pTouch_E->contact = contacts[pTouch_E->iFirstContact + touchContacts_[iTouch]];
        //if (touches_E.Element[0].contact.type == RVLMOTION_TOUCH_CONTACT_TYPE_POINT_PLANE &&
        //    touches_E.Element[0].contact.iToolFeature == 6 && touches_E.Element[0].contact.iEnvFeature.a == 3 && touches_E.Element[0].contact.iEnvFeature.b == 4 &&
        //    touches_E.Element[1].contact.type == RVLMOTION_TOUCH_CONTACT_TYPE_POINT_PLANE &&
        //    touches_E.Element[1].contact.iToolFeature == 6 && touches_E.Element[1].contact.iEnvFeature.a == 3 && touches_E.Element[1].contact.iEnvFeature.b == 4)
        //    int debug = 0;
        LM(x, touches_E, x, &E);
        if (iTouchContactCombination == 0 || E.E < minE.E)
        {
            minE = E;
            memcpy(xOpt, x, RVLMOTION_TOUCH_NUM_PARAMS * sizeof(float));
            pTouch_E = touches_E.Element;
            for (iTouch = 0; iTouch < touches_E.n; iTouch++, pTouch_E++)
                optimalContacts[iTouch] = pTouch_E->iFirstContact + touchContacts_[iTouch];
        }
        if (iTouchContactCombination % 1000 == 0 && iTouchContactCombination > 0)
            printf("%d/%d\n", iTouchContactCombination, touchContactCombinations.h);
    }
    for (iTouch = 0; iTouch < touches_E.n; iTouch++)
    {
        pTouch_E = touches_E.Element + iTouch;
        printf("touch %d: ", iTouch);
        MOTION::Contact contact;
        if (pTouch_E->nContacts > 0)
        {
            contact = contacts[optimalContacts[iTouch]];
            if (contact.type == RVLMOTION_TOUCH_CONTACT_TYPE_POINT_PLANE)
                printf("vertex %d panel %d face %d\n", contact.iToolFeature, contact.iEnvFeature.a, contact.iEnvFeature.b);
            else
                printf("tool_edge %d panel %d edge %d\n", contact.iToolFeature, contact.iEnvFeature.a, contact.iEnvFeature.b);
        }
        else
            printf("no contact\n");
    }
    printf("E=%f ravg=%f gmax=%f Ex=%f\n", minE.E, minE.ravg, minE.gmax, minE.Ex);
    delete[] optimalContacts;
    delete[] touchContactCombinations.Element;

    // Only for debugging purpose!!!

    //float EOpt = 0.0f;
    //for (i = 0; i < RVLMOTION_TOUCH_NUM_PARAMS; i++)
    //    EOpt += xOpt[i] * xOpt[i] / varx[i];
    //EOpt *= alpha;

    return true;
}

void Touch::Reconstruct(
    float* P_E,
    MOTION::TouchModel *pModel_x,
    float* P_E_x,
    float* P_C_x,
    float* invKxIn)
{
    float* invKx;
    if (invKxIn)
        invKx = invKxIn;
    else
    {
        invKx = new float[9];
        InvIntrinsicCameraMatrix(pModel_x->K, invKx);
    }

    // Compute P_C.

    float P_C[3];
    float V3Tmp[3];
    RVLINVTRANSF3(P_E, model_gt.pose_C_E.R, model_gt.pose_C_E.t, P_C, V3Tmp);

    // Q <- homogeneous coordinates of the projection of P_C onto the image

    float Q[3];
    RVLMULMX3X3VECT(model_gt.K, P_C, Q);
    RVLSCALE2VECTOR2(Q, Q[2], Q);
    Q[2] = 1.0f;

    // disp <- disparity of P_C

    float zn = model_gt.kappazn / model_gt.kappa;
    float disp = model_gt.kappa * (1.0f - zn / P_C[2]);

    // P_C_x <- 3D reconstruction from Q and disp

    float znx = pModel_x->kappazn / pModel_x->kappa;
    float zx = znx / (1.0f - disp / pModel_x->kappa);
    RVLMULMX3X3VECT(invKx, Q, P_C_x);
    RVLSCALE3VECTOR(P_C_x, zx, P_C_x);

    // Compute P_E_x from P_C_x using T_C_E.

    RVLTRANSF3(P_C_x, pModel_x->pose_C_E.R, pModel_x->pose_C_E.t, P_E_x);

    //

    if (invKxIn == NULL)
        delete[] invKx;
}

void Touch::SetVerticesAndPlanes(
    Array<MOTION::PlanarSurface> surfaces_,
    Array<MOTION::Vertex> vertices_,
    Pose3D pose,
    MOTION::TouchModel* pModel)
{
    int iSurface;
    MOTION::PlanarSurface* pSurface = surfaces_.Element;
    MOTION::Plane Plane_E;
    MOTION::Plane* pPlane_e = pModel->plane;
    for (iSurface = 0; iSurface < surfaces_.n; iSurface++, pSurface++, pPlane_e++)
        RVLPLANETRANSF3(pSurface->plane.N, pSurface->plane.d, pose.R, pose.t, pPlane_e->N, pPlane_e->d);

    int iVertex;
    MOTION::Vertex* pVertex = vertices_.Element;
    Vector3<float>* pVertex_e = pModel->vertex;
    for (iVertex = 0; iVertex < vertices_.n; iVertex++, pVertex++, pVertex_e++)
        RVLTRANSF3(pVertex->P, pose.R, pose.t, pVertex_e->Element);
}

void Touch::UpdateVerticesAndPlanes(MOTION::TouchModel* pModel)
{
    // Vertices.

    int iVertex;
    MOTION::Vertex* pVertex = vertices.Element;
    Vector3<float>* pVertexSrc = pModel->vertex;
    SolidVertex* pVertexTgt;
    int i;
    Pair<int, int>* pSolidVertexIdx;
    for (iVertex = 0; iVertex < vertices.n; iVertex++, pVertex++, pVertexSrc++)
    {
        pSolidVertexIdx = pVertex->solidVertices.Element;
        for (i = 0; i < pVertex->solidVertices.n; i++, pSolidVertexIdx++)
        {
            pVertexTgt = envSolidx.solids[pSolidVertexIdx->a]->vertices.Element + pSolidVertexIdx->b;
            RVLCOPY3VECTOR(pVertexSrc->Element, pVertexTgt->P);
        }
    }

    // Planes.

    int iSurface;
    MOTION::PlanarSurface* pSurface = surfaces.Element;
    MOTION::Plane* pPlaneSrc = pModel->plane;
    RECOG::VN_::Feature* pFeature;
    SolidFace* pFace;
    Pair<int, int>* pSolidFaceIdx;
    for (iSurface = 0; iSurface < surfaces.n; iSurface++, pSurface++, pPlaneSrc++)
    {
#ifdef RVLMOTION_TOUCH_VN
        for (i = 0; i < pSurface->VNFeatures.n; i++)
        {
            pFeature = pModel->pVNEnv->featureArray.Element + pSurface->VNFeatures.Element[i];
            RVLCOPY3VECTOR(pPlaneSrc->N, pFeature->N);
            pFeature->d = pPlaneSrc->d;
        }
#endif
        pSolidFaceIdx = pSurface->solidFaces.Element;
        for (i = 0; i < pSurface->solidFaces.n; i++, pSolidFaceIdx++)
        {
            pFace = envSolidx.solids[pSolidFaceIdx->a]->faces.Element + pSolidFaceIdx->b;
            RVLCOPY3VECTOR(pPlaneSrc->N, pFace->N);
            pFace->d = pPlaneSrc->d;
        }
    }
}

void Touch::UpdateDoorAxis(MOTION::TouchModel* pModel0)
{
    //CorrectVertex(pModel0->pose_A_E.t, )
}

void Touch::UpdateDoorOrientation(MOTION::TouchModel* pModel)
{
    float R_E_A_e[9];
    float* X_A_E_e = R_E_A_e;
    float* Y_A_E_e = R_E_A_e + 3;
    float* Z_A_E_e = R_E_A_e + 6;
    float* N = pModel->plane[doorRefSurfaceIdx].N;
    RVLNEGVECT3(N, X_A_E_e);

    // Computation of Z_A_E_e according to DOK-2021-02-3889_TR4.1, Correction of Door Parameters.

    float dP[3];
    RVLDIF3VECTORS(pModel->PAxis_E, pModel->pose_A_E.t, dP);
    float absN[3];
    RVLSET3VECTOR(absN, RVLABS(N[0]), RVLABS(N[1]), RVLABS(N[2]));
    int i1 = (absN[0] > absN[1] ? 0 : 1);
    int i2 = (absN[i1] > absN[2] ? i1 : 2);
    int i0 = (i2 + 1) % 3;
    i1 = (i2 + 2) % 3;
    float MM[4];
    MM[0] = N[i1] * N[i1] + N[i2] * N[i2];
    MM[1] = MM[2] = -N[i0] * N[i1];
    MM[3] = N[i0] * N[i0] + N[i2] * N[i2];
    float V2Tmp[2];
    float a0 = N[i0] / N[i2];
    float a1 = N[i1] / N[i2];
    V2Tmp[0] = dP[i0] - a0 * dP[i2];
    V2Tmp[1] = dP[i1] - a1 * dP[i2];
    float V2Tmp_[2];
    RVLMULMX2X2VECT(MM, V2Tmp, V2Tmp_);
    Z_A_E_e[i0] = V2Tmp_[0];
    Z_A_E_e[i1] = V2Tmp_[1];
    Z_A_E_e[i2] = -a0 * V2Tmp_[0] - a1 * V2Tmp_[1];
    float fTmp;
    RVLNORM3(Z_A_E_e, fTmp);

    //

    RVLCROSSPRODUCT3(Z_A_E_e, X_A_E_e, Y_A_E_e);
    RVLCOPYMX3X3T(R_E_A_e, pModel->pose_A_E.R);
}

void Touch::UpdateEnvironmentModel(
    MOTION::TouchModel* pModel0,
    float* x,
    MOTION::TouchModel* pModelx)
{
    float gz = x[4];
    float hz = x[5];

    // Corrected model.

    Correct(pModel0, x, pModelx);

    // Compute a, a_, b, b_, D and D.inv

    float a[3], a_[3];
    float b, b_;
    float D[9], invD[9];
    float M[9], V[3];
    AuxParams(pModel0, pModelx, gz, hz, a, b, a_, b_, D, invD, M, V);

    //// a <- gz / (kappazne) * z_E_C

    //fTmp = gz / pModel0->kappazn;
    //float Z_C_E_x[3];
    //RVLCOPYCOLMX3X3(pModelx->pose_C_E.R, 2, Z_C_E_x);
    //RVLSCALE3VECTOR(Z_C_E_x, fTmp, a);

    //// b <- a * t_E_C_x + 1 + hz / kappazne

    //b = RVLDOTPRODUCT3(a, pModelx->pose_C_E.t) + 1.0f + hz / pModel0->kappazn;

    //// invD <- pModel0->pose_C_E.R * pModel0->K.inv() * pModelx->K * pModelx->pose_C_E.R.T

    //float M3x3Tmp1[9];
    //RVLMXMUL3X3T2(pModelx->K, pModelx->pose_C_E.R, M3x3Tmp1);
    //float invK0[9];
    //InvIntrinsicCameraMatrix(pModel0->K, invK0);
    //float M3x3Tmp2[9];
    //RVLMXMUL3X3(pModel0->pose_C_E.R, invK0, M3x3Tmp2);
    //RVLMXMUL3X3(M3x3Tmp2, M3x3Tmp1, invD);

    // Only for debugging purpose!!!

    //{
    //    float P_E[3];
    //    RVLSET3VECTOR(P_E, 0.5f, 0.6f, 1.34f);
    //    float P_E_x[3];
    //    float P_C_x[3];
    //    Reconstruct(P_E, &modelx, P_E_x, P_C_x);
    //    float P_E_e[3];
    //    float P_C_e[3];
    //    Reconstruct(P_E, pModel0, P_E_e, P_C_e);
    //    fTmp = b - RVLDOTPRODUCT3(a, P_E_x);
    //    float V3Tmp[3];
    //    RVLDIF3VECTORS(P_E_x, pModelx->pose_C_E.t, V3Tmp);
    //    RVLSCALE3VECTOR2(V3Tmp, fTmp, V3Tmp);
    //    float P_E_e_[3];
    //    RVLMULMX3X3VECT(invD, V3Tmp, P_E_e_);
    //    RVLSUM3VECTORS(P_E_e_, pModel0->pose_C_E.t, P_E_e_);
    //    RVLDIF3VECTORS(P_E_e, P_E_e_, V3Tmp);
    //    float err = sqrt(RVLDOTPRODUCT3(V3Tmp, V3Tmp));
    //    int debug = 0;
    //}

    // Update VN model.

    //int iFeature;
    //RECOG::VN_::Feature* pFeatureSrc, * pFeatureTgt;
    //MOTION::Plane* plane_E = new MOTION::Plane[pModel0->pVNEnv->featureArray.n];
    //MOTION::Plane* pPlane_E;
    //for (iFeature = 0; iFeature < pModel0->pVNEnv->featureArray.n; iFeature++)
    //{
    //    pFeatureSrc = pModel0->pVNEnv->featureArray.Element + iFeature;
    //    pFeatureTgt = pModelx->pVNEnv->featureArray.Element + iFeature;

    //    // pFeatureTgt->N <- (M.T * pFeatureSrc->N + pFeatureSrc->d * a.T).norm

    //    RVLMULMX3X3TVECT(M, pFeatureSrc->N, pFeatureTgt->N);        
    //    RVLSCALE3VECTOR(a, pFeatureSrc->d, V3Tmp);
    //    RVLSUM3VECTORS(pFeatureTgt->N, V3Tmp, pFeatureTgt->N);
    //    fTmp = sqrt(RVLDOTPRODUCT3(pFeatureTgt->N, pFeatureTgt->N));
    //    RVLSCALE3VECTOR2(pFeatureTgt->N, fTmp, pFeatureTgt->N);

    //    // pFeatureTgt->d <- (b * pFeatureSrc->d + pFeatureSrc->N.T * V) / norm(M * pFeatureSrc->N + pFeatureSrc->d * a.T)

    //    pFeatureTgt->d = (b * pFeatureSrc->d + RVLDOTPRODUCT3(pFeatureSrc->N, V)) / fTmp;
    //}

    // Update vertices.

    int iVertex;
    Vector3<float>* pVertexSrc = pModel0->vertex;
    Vector3<float>* pVertexTgt = pModelx->vertex;
    for (iVertex = 0; iVertex < vertices.n; iVertex++, pVertexSrc++, pVertexTgt++)
        CorrectVertex(pVertexSrc->Element, a_, b_, D, &(pModel0->pose_C_E), &(pModelx->pose_C_E), pVertexTgt->Element);
    //{
    //    // pModelx->vertex[iVertex] <- 1.0 / (a_ * pModel0->vertex[iVertex] + b_) * D * (pModel0->vertex[iVertex] - pModel0->pose_C_E.t) + pModelx->pose_C_E.t

    //    RVLDIF3VECTORS(pVertexSrc->Element, pModel0->pose_C_E.t, V3Tmp);
    //    RVLMULMX3X3VECT(D, V3Tmp, pVertexTgt->Element);
    //    fTmp = RVLDOTPRODUCT3(a_, pVertexSrc->Element) + b_;
    //    RVLSCALE3VECTOR2(pVertexTgt->Element, fTmp, pVertexTgt->Element);
    //    RVLSUM3VECTORS(pVertexTgt->Element, pModelx->pose_C_E.t, pVertexTgt->Element);
    //}

    // Update surfaces.

    int iSurface;
    MOTION::Plane* pPlaneSrc = pModel0->plane;
    MOTION::Plane* pPlaneTgt = pModelx->plane;
    for (iSurface = 0; iSurface < surfaces.n; iSurface++, pPlaneSrc++, pPlaneTgt++)
        CorrectPlane(pPlaneSrc, a, b, M, V, pPlaneTgt);
    //{
    //    // pPlaneTgt->N <- (M.T * pPlaneSrc->N + pPlaneSrc->d * a.T).norm

    //    RVLMULMX3X3TVECT(M, pPlaneSrc->N, pPlaneTgt->N);
    //    RVLSCALE3VECTOR(a, pPlaneSrc->d, V3Tmp);
    //    RVLSUM3VECTORS(pPlaneTgt->N, V3Tmp, pPlaneTgt->N);
    //    fTmp = sqrt(RVLDOTPRODUCT3(pPlaneTgt->N, pPlaneTgt->N));
    //    RVLSCALE3VECTOR2(pPlaneTgt->N, fTmp, pPlaneTgt->N);

    //    // pPlaneTgt->d <- (b * pPlaneSrc->d + pPlaneSrc->N.T * V) / norm(M * pPlaneSrc->N + pPlaneSrc->d * a.T)

    //    pPlaneTgt->d = (b * pPlaneSrc->d + RVLDOTPRODUCT3(pPlaneSrc->N, V)) / fTmp;
    //}

    // Update door axis.

    if (bDoor)
    {
        float V3Tmp[3];
        CorrectVertex(pModel0->pose_A_E.t, a_, b_, D, &(pModel0->pose_C_E), &(pModelx->pose_C_E), pModelx->pose_A_E.t);
        RVLCOPYCOLMX3X3(pModel0->pose_A_E.R, 2, V3Tmp);
        RVLSUM3VECTORS(pModel0->pose_A_E.t, V3Tmp, V3Tmp);
        CorrectVertex(V3Tmp, a_, b_, D, &(pModel0->pose_C_E), &(pModelx->pose_C_E), pModelx->PAxis_E);
    }

    // Update vertices and planes.

    UpdateVerticesAndPlanes(pModelx);

    // Test vertex-plane consistency.

    //TestVertexPlaneConsistency();
}

void Touch::UpdateEnvironmentVNModel(
    MOTION::TouchModel* pModel0,
    float *x,
    MOTION::TouchModel* pModelx)
{
    float gz = x[4];
    float hz = x[5];

    float a[3], a_[3];
    float b, b_;
    float D[9], invD[9];
    float M[9], V[3];
    AuxParams(pModel0, pModelx, gz, hz, a, b, a_, b_, D, invD, M, V);

    int iFeature;    
    MOTION::Plane* plane_E = new MOTION::Plane[pModel0->pVNEnv->featureArray.n];
    MOTION::Plane* pPlane_E;
    float V3Tmp[3];
    float fTmp;
    RECOG::VN_::Feature* pFeatureSrc = pModel0->pVNEnv->featureArray.Element;
    RECOG::VN_::Feature* pFeatureTgt = pModelx->pVNEnv->featureArray.Element;
    MOTION::Plane planeSrc, planeTgt;
    for (iFeature = 0; iFeature < pModel0->pVNEnv->featureArray.n; iFeature++, pFeatureSrc++, pFeatureTgt++)
    {
        RVLCOPY3VECTOR(pFeatureSrc->N, planeSrc.N);
        planeSrc.d = pFeatureSrc->d;
        CorrectPlane(&planeSrc, a, b, M, V, &planeTgt);
        RVLCOPY3VECTOR(planeTgt.N, pFeatureTgt->N);
        pFeatureTgt->d = planeTgt.d;
    }
}

void Touch::RndTouchPoint(MOTION::TouchPoint& touchPt)
{
    // Choose a panel.

    if (touchPt.iPanel < 0)
        RVLRND(4, rndVal.Element, 1000000, iRndVal, touchPt.iPanel);
    Solid* pPanel = envSolidx.solids[touchPt.iPanel];

    // Choose a face.

    SolidFace* pFace;
    int i;
    if (touchPt.iFace < 0)
    {
        float maxArea = 0.0f;
        pFace = pPanel->faces.Element;
        int iFace;
        for (iFace = 0; iFace < pPanel->faces.n; iFace++, pFace++)
            if (pFace->area > maxArea)
                maxArea = pFace->area;
        Array<int> maxFaces;
        int maxFacesMem[6];
        maxFaces.Element = maxFacesMem;
        maxFaces.n = 0;
        pFace = pPanel->faces.Element;
        for (iFace = 0; iFace < pPanel->faces.n; iFace++, pFace++)
            if (pFace->area > 0.99f * maxArea)
                maxFaces.Element[maxFaces.n++] = iFace;
        RVLRND(maxFaces.n, rndVal.Element, 1000000, iRndVal, i);
        touchPt.iFace = maxFaces.Element[i];
    }
    pFace = pPanel->faces.Element + touchPt.iFace;

    // Choose the reference edge.

    float maxLen = 0.0f;
    int iEdge0, iEdge;
    iEdge0 = iEdge = pFace->iEdge;
    SolidEdge* pEdge;
    do
    {
        pEdge = pPanel->edges.Element + iEdge;
        if (pEdge->len > maxLen)
            maxLen = pEdge->len;
        iEdge = pEdge->iNext;
    } while (iEdge != iEdge0);
    iEdge0 = iEdge = pFace->iEdge;
    Array<int> maxEdges;
    int maxEdgesMem[6];
    maxEdges.Element = maxEdgesMem;
    maxEdges.n = 0;
    do
    {
        pEdge = pPanel->edges.Element + iEdge;
        if (pEdge->len > 0.99f * maxLen)
            maxEdges.Element[maxEdges.n++] = iEdge;
        iEdge = pEdge->iNext;
    } while (iEdge != iEdge0);
    RVLRND(maxEdges.n, rndVal.Element, 1000000, iRndVal, i);
    touchPt.iEdge = maxEdges.Element[i];

    // Target point on the panel surface.

    touchPt.u = RealPseudoRand<float>(rndVal, iRndVal);
    touchPt.v = RealPseudoRand<float>(rndVal, iRndVal);
}

void Touch::RefPts()
{
    Pair<int, int> iRefVertex[RVLMOTION_TOUCH_NUM_REF_PTS];
    Pair<int, int>* pRefVertexIdx = iRefVertex;
    float bound[6];
    int i, j;
    SolidVertex* pVertex = envSolidx.solids[0]->vertices.Element;
    for (i = 0; i < 3; i++)
    {
        for (j = 0; j < 2; j++, pRefVertexIdx++)
        {
            bound[2 * i + j] = pVertex->P[i];
            pRefVertexIdx->a = pRefVertexIdx->b = 0;
        }
    }
    int iPart, iVertex;
    Solid* pPart;
    for (iPart = 0; iPart < envSolidx.solids.size(); iPart++)
    {
        pPart = envSolidx.solids[iPart];
        pVertex = pPart->vertices.Element;
        for (iVertex = 0; iVertex < pPart->vertices.n; iVertex++, pVertex++)
        {
            for (i = 0; i < 3; i++)
            {
                if (pVertex->P[i] < bound[2 * i])
                {
                    bound[2 * i] = pVertex->P[i];
                    pRefVertexIdx = iRefVertex + 2 * i;
                    pRefVertexIdx->a = iPart;
                    pRefVertexIdx->b = iVertex;
                }
                else if(pVertex->P[i] > bound[2 * i + 1])
                {
                    bound[2 * i + 1] = pVertex->P[i];
                    pRefVertexIdx = iRefVertex + 2 * i + 1;
                    pRefVertexIdx->a = iPart;
                    pRefVertexIdx->b = iVertex;
                }
            }
        }
    }
    RVL_DELETE_ARRAY(refPts.Element);
    refPts.Element = new MOTION::TouchData[3 * RVLMOTION_TOUCH_NUM_REF_PTS];
    MOTION::TouchData* pRefPt = refPts.Element;
    pRefVertexIdx = iRefVertex;
    int iEdge, iEdge0;
    SolidEdge* pEdge;
    int iRefFace;
    for (i = 0; i < 6; i++, pRefVertexIdx++)
    {
        pPart = envSolidx.solids[pRefVertexIdx->a];
        pVertex = pPart->vertices.Element + pRefVertexIdx->b;
        iEdge = iEdge0 = pVertex->iEdge;
        iRefFace = 0;
        do
        {
            RVLCOPY3VECTOR(pVertex->P, pRefPt->pose.t);
            pEdge = pPart->edges.Element + iEdge;
            pRefPt->contact.iEnvFeature.a = pRefVertexIdx->a;
            pRefPt->contact.iEnvFeature.b = pEdge->iFace;
            pRefPt->contact.type = RVLMOTION_TOUCH_CONTACT_TYPE_POINT_PLANE;
            pRefPt++;
            iRefFace++;
            iEdge = pPart->edges.Element[pEdge->iPrev].iTwin;
        } while (iEdge != iEdge0 && iRefFace < 3);
    }
    refPts.n = pRefPt - refPts.Element;
}

void Touch::PlanTouch(
    MOTION::TouchPoint touchPt,
    Pose3D& initPose,
    Array<Vector3<float>>& path,
    float* PTgt)
{
    // Parameters.

    float a = 0.03f;    // m
    float h = 0.03f;    // m
    float offset = 0.05f;    // m
    float dist = 0.005f;   // m
    float parallelMoveLen = 0.5f;   // m
    float orthogonalMoveLen = 0.10f; // m

    // 

    Solid *pPanel = envSolidx.solids[touchPt.iPanel];
    SolidFace* pFace = pPanel->faces.Element + touchPt.iFace;
    SolidEdge* pEdge = pPanel->edges.Element + touchPt.iEdge;

    float* P0, * P1, * P2;
    SolidEdge* pEdge_;
    P0 = pPanel->vertices.Element[pEdge->iVertex].P;
    pEdge_ = pPanel->edges.Element + pEdge->iNext;
    P1 = pPanel->vertices.Element[pEdge_->iVertex].P;
    pEdge_ = pPanel->edges.Element + pEdge->iPrev;
    P2 = pPanel->vertices.Element[pEdge_->iVertex].P;
    float V1[3], V2[3];
    RVLDIF3VECTORS(P1, P0, V1);
    RVLNORM3(V1, pEdge->len);
    RVLDIF3VECTORS(P2, P0, V2);
    RVLNORM3(V2, pEdge_->len);
    float p = offset + (pEdge->len - 2.0f * offset) * touchPt.u;
    float q = offset + (pEdge_->len - 2.0f * offset) * touchPt.v;
    float V3Tmp[3];
    RVLSCALE3VECTOR(V1, p, V3Tmp);
    RVLSUM3VECTORS(P0, V3Tmp, PTgt);
    RVLSCALE3VECTOR(V2, q, V3Tmp);
    RVLSUM3VECTORS(PTgt, V3Tmp, PTgt);

    // Tool orientation.

    float R_E_Bk[9];
    float* X_Bk_E = R_E_Bk;
    float* Y_Bk_E = R_E_Bk + 3;
    float* Z_Bk_E = R_E_Bk + 6;
    RVLNEGVECT3(pFace->N, X_Bk_E);
    RVLCOPY3VECTOR(V1, Y_Bk_E);
    RVLCROSSPRODUCT3(X_Bk_E, Y_Bk_E, Z_Bk_E);
    float RTilt[9];
    float toolTilt = toolTiltDeg * DEG2RAD;
    float cs = cos(toolTilt);
    float sn = sin(toolTilt);
    RVLROTY(cs, sn, RTilt);
    RVLMXMUL3X3T1(R_E_Bk, RTilt, initPose.R);
       
    // Via point.

    path.n = 0;
    P1 = path.Element[path.n++].Element;
    float TCP[2];
    TCP[0] = 0.5f * a;
    TCP[1] = h;
    float RTilt2D[4];
    sn = -sn;
    RVLROT2D(cs, sn, RTilt2D);
    float TCPTilted[2];
    RVLMULMX2X2VECT(RTilt2D, TCP, TCPTilted);
    float fTmp = dist + TCPTilted[0];
    RVLSCALE3VECTOR(pFace->N, fTmp, V3Tmp);
    RVLSUM3VECTORS(PTgt, V3Tmp, P1);

    // End point.

    P2 = path.Element[path.n++].Element;
    RVLSCALE3VECTOR(pFace->N, orthogonalMoveLen, V3Tmp);
    RVLDIF3VECTORS(P1, V3Tmp, P2);

    // Start point.

    RVLSCALE3VECTOR(Z_Bk_E, parallelMoveLen, V3Tmp);
    RVLDIF3VECTORS(P1, V3Tmp, initPose.t);
}

void Touch::SimulateMove(
    Pose3D initPose,
    Array<Vector3<float>> path,
    Pose3D& finalPose,
    float* V,
    int& iLastSegment)
{
    float* P1 = initPose.t;
    int iSegment;
    float* P2;
    finalPose = initPose;
    float t;
    float dist;
    for (iSegment = 0; iSegment < path.n; iSegment++)
    {
        P2 = path.Element[iSegment].Element;
        RVLDIF3VECTORS(P2, P1, V);
        dist = sqrt(RVLDOTPRODUCT3(V, V));
        toolMoved.Move(&(tool.solid), &finalPose);
        t = toolMoved.FreeMove(V, &envSolid_E);
        RVLSCALE3VECTOR2(V, dist, V);
        if (dist - t > 1e-4)
        {
            float V3Tmp[3];
            RVLSCALE3VECTOR(V, t, V3Tmp);
            RVLSUM3VECTORS(finalPose.t, V3Tmp, finalPose.t);
            break;
        }
        P1 = P2;
        RVLCOPY3VECTOR(P2, finalPose.t);
    }
    iLastSegment = iSegment;
}

void Touch::SceneBBox(
    MOTION::TouchModel* pModel,
    Box<float>* pBBox)
{
    Vector3<float> *pVertex_e = model_e.vertex;
    InitBoundingBox<float>(pBBox, pVertex_e->Element);
    pVertex_e++;
    for (int iVertex = 1; iVertex < vertices.n; iVertex++, pVertex_e++)
        UpdateBoundingBox<float>(pBBox, pVertex_e->Element);
}

void Touch::InitVisualizer(
    Visualizer* pVisualizerIn,
    char* cfgFileName)
{
    MOTION::InitVisualizer(pVisualizerIn, pVisualizationData, pMem0);
    pVisualizationData->paramList.m_pMem = pMem0;
    RVLPARAM_DATA* pParamData;
    pVisualizationData->paramList.Init();
    pParamData = pVisualizationData->paramList.AddParam("Touch.Visualize_optimization", RVLPARAM_TYPE_BOOL, &(pVisualizationData->bOptimization));
    pVisualizationData->paramList.LoadParams(cfgFileName);
}

void Touch::SetVisualizeOptimization(bool bVisualizeOptimization)
{
    pVisualizationData->bOptimization = bVisualizeOptimization;
}

bool Touch::Intersection(
    float* N1,
    float* N2,
    float* N3,
    float* N4)
{
    // A <- negated matrix whose columns are N2, N3 and N4

    cv::Mat cvA(3, 3, CV_32FC1);
    float* A = (float*)(cvA.data);
    RVLCOPYTOCOL3(N2, 0, A);
    RVLCOPYTOCOL3(N3, 1, A);
    RVLCOPYTOCOL3(N4, 2, A);
    int i;
    for (i = 0; i < 9; i++)
        A[i] = -A[i];

    // b <- N1

    cv::Mat cvb(3, 1, CV_32FC1);
    float* b = (float*)(cvb.data);
    RVLCOPY3VECTOR(N1, b);

    // x <- solution to Ax = b

    cv::Mat cvx(3, 1, CV_32FC1);
    float* x = (float*)(cvx.data);
    cv::solve(cvA, cvb, cvx);

    // If all elements of x are >= 0, then the intersection is an empty set.

    for (i = 0; i < 3; i++)
        if (x[i] < 0)
            return true;
    return false;
}

void Touch::Constants()
{
    stdl[0] = stdgxyk * camera.fu;
    stdl[1] = stdgxyk * camera.fv;
    stdl[2] = stdhxyk * camera.uc;
    stdl[3] = stdhxyk * camera.vc;
    stdgz = stdgzk * kappa;
    stdhz = stdhzk * kappa * zn;
    stdphirad = DEG2RAD * stdphi;
    int i;
    for(i = 0; i < 4; i++)
        varx[i] = stdl[i] * stdl[i];
    varx[4] = stdgz * stdgz;
    varx[5] = stdhz * stdhz;
    for (i = 0; i < 3; i++)
    {
        varx[6 + i] = stdphirad * stdphirad;
        varx[9 + i] = stds * stds;
        varx[12 + i] = stdc * stdc;
    }
}

void Touch::AuxParams(
    MOTION::TouchModel* pModel0,
    MOTION::TouchModel* pModelx,
    float gz,
    float hz,
    float* a,
    float& b,
    float* a_,
    float& b_,
    float* D,
    float* invD,
    float* M,
    float* V)
{
    // a <- gz / pModel0->kappazn * pModelx->pose_C_E.z

    float fTmp = gz / pModel0->kappazn;
    float Z_C_E_x[3];
    RVLCOPYCOLMX3X3(pModelx->pose_C_E.R, 2, Z_C_E_x);
    RVLSCALE3VECTOR(Z_C_E_x, fTmp, a);

    // b <- a * pModelx->pose_C_E.t + 1 + hz / pModel0->kappazn

    b = RVLDOTPRODUCT3(a, pModelx->pose_C_E.t) + 1.0f + hz / pModel0->kappazn;

    // invD <- pModel0->pose_C_E.R * pModel0->K.inv() * pModelx->K * pModelx->pose_C_E.R.T

    float M3x3Tmp1[9];
    RVLMXMUL3X3T2(pModelx->K, pModelx->pose_C_E.R, M3x3Tmp1);
    float invK0[9];
    InvIntrinsicCameraMatrix(pModel0->K, invK0);
    float M3x3Tmp2[9];
    RVLMXMUL3X3(pModel0->pose_C_E.R, invK0, M3x3Tmp2);
    RVLMXMUL3X3(M3x3Tmp2, M3x3Tmp1, invD);

    // a_ <- gz / pModelx->kappazn * pModel0->pose_C_E.z.T

    float Z_C_E_0[3];
    RVLCOPYCOLMX3X3(pModel0->pose_C_E.R, 2, Z_C_E_0);
    fTmp = gz / pModelx->kappazn;
    RVLSCALE3VECTOR(Z_C_E_0, fTmp, a_);

    // b_ <- -a_ * pModel0->pose_C_E.t + pModel0->kappazn / pModelx->kappazn

    b_ = -RVLDOTPRODUCT3(a_, pModel0->pose_C_E.t) + pModel0->kappazn / pModelx->kappazn;

    // D <- pModelx->pose_C_E.R * pModelx->K.inv() * pModel0->K * pModel0->pose_C_E.R.T

    RVLMXMUL3X3T2(pModel0->K, pModel0->pose_C_E.R, M3x3Tmp1);
    float invKx[9];
    InvIntrinsicCameraMatrix(pModelx->K, invKx);
    RVLMXMUL3X3(pModelx->pose_C_E.R, invKx, M3x3Tmp2);
    RVLMXMUL3X3(M3x3Tmp2, M3x3Tmp1, D);

    // M <- invD - pModel0->pose_C_E.t * a

    RVLMULVECT3VECT3T(pModel0->pose_C_E.t, a, M);
    RVLDIFMX3X3(invD, M, M);

    // V <- invD * pModelx->pose_C_E.t - b * pModel0->pose_C_E.t

    RVLMULMX3X3VECT(invD, pModelx->pose_C_E.t, V);
    float V3Tmp[3];
    RVLSCALE3VECTOR(pModel0->pose_C_E.t, b, V3Tmp);
    RVLDIF3VECTORS(V, V3Tmp, V);
}

void Touch::VisionVertex(
    float* PSrc,
    float *a,
    float b,
    float* invD,
    Pose3D pose_C_E,
    Pose3D pose_C_E_e,
    float* PTgt)
{
    float P_E[3];

    // P_E <- vertices.Elment[iVertex].P transformed to r.f. E

    RVLTRANSF3(PSrc, pose_W_E.R, pose_W_E.t, P_E);

    // model_e.vertex[iVertex].Element <- 1 / (b - a * P_E) * D.inv * (P_E - t_C_E) + t_C_E_e

    float fTmp = b - RVLDOTPRODUCT3(a, P_E);
    float V3Tmp[3];
    RVLDIF3VECTORS(P_E, pose_C_E.t, V3Tmp);
    RVLMULMX3X3VECT(invD, V3Tmp, PTgt);
    RVLSCALE3VECTOR2(PTgt, fTmp, PTgt);
    RVLSUM3VECTORS(PTgt, pose_C_E_e.t, PTgt);
}

void Touch::VisionPlane(
    MOTION::Plane *pPlaneSrc,
    float *a_,
    float b_,
    float *D,
    Pose3D *pPose_C_E,
    Pose3D* pPose_C_E_e,
    MOTION::Plane* pPlaneTgt)
{
    // r <- pPlaneSrc->d - pPlaneSrc->N.T * pose_C_E.t 

    float r = pPlaneSrc->d - RVLDOTPRODUCT3(pPlaneSrc->N, pPose_C_E->t);

    // model_e.plane[iSurface].N <- (D.T * pPlaneSrc->N - r * a_.T).norm

    float V3Tmp[3];
    RVLMULMX3X3TVECT(D, pPlaneSrc->N, V3Tmp);
    RVLSCALE3VECTOR(a_, r, pPlaneTgt->N);
    RVLDIF3VECTORS(V3Tmp, pPlaneTgt->N, pPlaneTgt->N);
    float fTmp;
    RVLNORM3(pPlaneTgt->N, fTmp);

    // model_e.plane[iSurface].d <- (b_ * r  + (pPlaneSrc->N.T * D * t_C_E_e) / norm(D.T * pPlaneSrc->N - r * a_.T)

    pPlaneTgt->d = (r * b_ + RVLDOTPRODUCT3(V3Tmp, pPose_C_E_e->t)) / fTmp;
}

void Touch::TestPlaneMapping(MOTION::TouchModel *pModel_x)
{
    float invKx[9];
    InvIntrinsicCameraMatrix(pModel_x->K, invKx);

    float sceneSize[3];
    BoxSize<float>(&(envSolid.bbox), sceneSize[0], sceneSize[1], sceneSize[2]);
    float P0bbox_W[3];
    RVLSET3VECTOR(P0bbox_W, envSolid.bbox.minx, envSolid.bbox.miny, envSolid.bbox.minz);
    float PTest_W[3];
    float PTest_C[3];
    float PTest_E[3];
    float PTest_C_e[3];
    float QTest[3];
    float PTest_E_e[3];
    float e_;
    int iFeature;
    for (iFeature = 0; iFeature < model_gt.pVNEnv->featureArray.n; iFeature++)
    {
        RECOG::VN_::Feature* pFeature = model_gt.pVNEnv->featureArray.Element + iFeature;
        RECOG::VN_::Feature* pFeaturex = pModel_x->pVNEnv->featureArray.Element + iFeature;
        float disp;
        float ze;
        float err;
        float V3Tmp[3], V3Tmp2[3];
        for (int i = 0; i < 3; i++)
        {
            // PTest_W <- a random point

            for (int j = 0; j < 3; j++)
                PTest_W[j] = sceneSize[j] * RealPseudoRand<float>(rndVal, iRndVal) + P0bbox_W[j];

            // Project PTest_W onto the plane pFeature.

            e_ = RVLDOTPRODUCT3(pFeature->N, PTest_W) - pFeature->d;
            RVLSCALE3VECTOR(pFeature->N, e_, V3Tmp);
            RVLDIF3VECTORS(PTest_W, V3Tmp, PTest_W);

            // Compute PTest_E.

            RVLTRANSF3(PTest_W, pose_W_E.R, pose_W_E.t, PTest_E);

            // Project PTest_E onto image and reconstruct it back using pModel_x.

            float PTest_E_x[3];
            Reconstruct(PTest_E, pModel_x, PTest_E_x, invKx);

            // Error.

            err = RVLDOTPRODUCT3(pFeaturex->N, PTest_E_x) - pFeaturex->d;
            printf("err=%f\n", err);
        }
    }
}

void Touch::TestVertexPlaneConsistency()
{
    Solid* pSolid;
    SolidEdge* pEdge;
    SolidFace* pFace;
    SolidVertex* pSolidVertex;
    int iEdge0, iEdge;
    float e;
    int i;
    for (int iSolid = 0; iSolid < envSolidx.solids.size(); iSolid++)
    {
        pSolid = envSolidx.solids[iSolid];
        for (i = 0; i < pSolid->vertices.n; i++)
        {
            pSolidVertex = pSolid->vertices.Element + i;
            iEdge = iEdge0 = pSolidVertex->iEdge;
            do
            {
                pEdge = pSolid->edges.Element + iEdge;
                pFace = pSolid->faces.Element + pEdge->iFace;
                e = RVLDOTPRODUCT3(pFace->N, pSolidVertex->P) - pFace->d;
                printf("error=%f\n", e);
                iEdge = pSolid->edges.Element[pEdge->iPrev].iTwin;
            } while (iEdge != iEdge0);
        }
    }
}

// Simundic
void Touch::CopyTouchModel(const RVL::MOTION::TouchModel& src, RVL::MOTION::TouchModel& dst) {
    // Copy POD and fixed-size arrays
    RVLCOPYMX3X3(src.pose_C_E.R, dst.pose_C_E.R);
    RVLCOPY3VECTOR(src.pose_C_E.t, dst.pose_C_E.t);

    dst.camera.fu = src.camera.fu;
    dst.camera.fv = src.camera.fv;
    dst.camera.uc = src.camera.uc;
    dst.camera.vc = src.camera.vc;
    dst.camera.w = src.camera.w;
    dst.camera.h = src.camera.h;

    RVLCOPYMX3X3(src.K, dst.K);

    dst.kappa = src.kappa;
    dst.kappazn = src.kappazn;

    RVLCOPY3VECTOR(src.TCP_E, dst.TCP_E);

    // ???
    dst.pVNEnv = src.pVNEnv;
    dst.d = src.d;
    SetVerticesAndPlanes(surfaces, vertices, pose_W_E, &model_e);

    // dst.plane = src.plane;
    // dst.vertex = src.vertex;
    
    RVLCOPYMX3X3(src.pose_A_E.R, dst.pose_A_E.R);
    RVLCOPY3VECTOR(src.pose_A_E.t, dst.pose_A_E.t);
    float z_E[3] = {src.pose_A_E.R[2], src.pose_A_E.R[5], src.pose_A_E.R[8]};
    RVLSUM3VECTORS(z_E, pose_A_E.t, dst.PAxis_E);
    // RVLCOPY3VECTOR(src.PAxis_E, dst.PAxis_E);
}


void Touch::RealExpCorrect(Array<MOTION::TouchData> &touches, 
    std::vector<MOTION::Contact> &contacts,
    Pose3D pose_Ek_E,
    float* V,
    Pose3D pose_A_E,
    Pose3D pose_E_0,
    Pose3D pose_0_S)
{
    RVLCOLORS   // For visualization.
    Visualizer* pVisualizer = pVisualizationData->pVisualizer;

    // u model_e i model_gt stavljam pose_A_E, pose_C_E, kappa, kappazn, K izvana.
    // kopiram model_gt u model_e
    CopyTouchModel(model_gt, model_e);

    // x definiram da je sve 0 (?) -- ovo je sad x_real i postavlja se i alocira izvana
    // float x[RVLMOTION_TOUCH_NUM_PARAMS];
    // for (int i = 0; i < RVLMOTION_TOUCH_NUM_PARAMS; i++)
    //     x[i] = 0.0f;
    //
    
    // tool.TCP -- treba definirati u odnosu na ks E // -- ne ovdje - samo  treba postavit translaciju -- postavljeno 

    toolMoved.Copy(&(tool.solid));
    toolMoved.Move(&(tool.solid), &pose_Ek_E);
    toolMoved.pVisualizer = pVisualizer;

    // Ovaj dio treba?
    envSolid_E.Copy(&envSolid);
    envSolid_E.Move(&envSolid, &pose_W_E);
    envSolid_E.Visualize(pVisualizer, black);

    // dobiti pose_Ek_E i V
    MOTION::TouchData touch;
    float xOpt[RVLMOTION_TOUCH_NUM_PARAMS];
    std::vector<MOTION::TouchData> touches_E;

    RVLCOPYMX3X3(pose_Ek_E.R, touch.pose.R);
    RVLCOPY3VECTOR(pose_Ek_E.t, touch.pose.t);
    // touch.pose = pose_Ek_E;
    RVLCOPY3VECTOR(V, touch.V);
    touch.iFirstContact = -1;
    touches_E.push_back(touch);
    touches.n = touches_E.size();
    touches.Element = touches_E.data();
    if (!Correction(x_real, touches, contacts, xOpt))
        return;

    UpdateEnvironmentModel(&model_e, xOpt, &model_x);
    UpdateDoorOrientation(&model_x);
    UpdateEnvironmentVNModel(&model_e, xOpt, &model_x);

    // Copy optimized parameters to the init ones. 
    memcpy(x_real, xOpt, RVLMOTION_TOUCH_NUM_PARAMS * sizeof(float));

    // W frame is center of the cabinet, E frame is the end-effector frame.
    // transform pVNEnv iz model_x u ks world
    // transformacija E u W i W u 0
    Pose3D pose_W_0, pose_W_S; 
    RVLCOMPTRANSF3D(pose_E_0.R, pose_E_0.t, pose_W_E.R, pose_W_E.t, pose_W_0.R, pose_W_0.t);
    RVLCOMPTRANSF3D(pose_0_S.R, pose_0_S.t, pose_W_0.R, pose_W_0.t, pose_W_S.R, pose_W_S.t);
    // model_x.pVNEnv->Transform(pose_W_S.R, pose_W_S.t); // -- za manipulator
    model_x.pVNEnv->Transform(pose_W_0.R, pose_W_0.t); // -- za manipulator

    // model_x.pose_A_E // ovo mi je rez
}

void Touch::setModelGTParams(Pose3D pose_A_E, Pose3D pose_C_E, float kappa, float kappazn, float *K)
{
    // Set model_gt parameters.
    RVLCOPYMX3X3(pose_A_E.R, model_gt.pose_A_E.R);
    RVLCOPY3VECTOR(pose_A_E.t, model_gt.pose_A_E.t);
    RVLCOPYMX3X3(pose_C_E.R, model_gt.pose_C_E.R);
    RVLCOPY3VECTOR(pose_C_E.t, model_gt.pose_C_E.t);
    model_gt.kappa = kappa;
    model_gt.kappazn = kappazn;
    RVLCOPYMX3X3(K, model_gt.K);
    float z_E[3] = {pose_A_E.R[2], pose_A_E.R[5], pose_A_E.R[8]};
    RVLSUM3VECTORS(z_E, pose_A_E.t, model_gt.PAxis_E);
}