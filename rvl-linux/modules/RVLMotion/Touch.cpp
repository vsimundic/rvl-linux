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
#include "cnpy.h"
#include "vtkNew.h"

// #define RVLMOTION_TOUCH_VN

using namespace RVL;
using namespace MOTION;

// Move to Util.

namespace RVL
{
    bool Line2DCircleIntersection(
        float a,
        float b,
        float c,
        float r,
        float *P1,
        float *P2)
    {
        float a_, b_;
        int ix, iy;
        if (RVLABS(a) >= RVLABS(b))
        {
            a_ = a;
            b_ = b;
            ix = 0;
            iy = 1;
        }
        else
        {
            a_ = b;
            b_ = a;
            ix = 1;
            iy = 0;
        }
        float a__ = a_ * a_ + b_ * b_;
        float b__ = 2.0f * b_ * c;
        float c__ = c * c - a_ * a_ * r * r;
        float dis = b__ * b__ - 4.0f * a__ * c__;
        if (dis < 0.0f)
            return false;
        float den = 2.0f * a__;
        float p = -b__ / den;
        float q = sqrt(dis) / den;
        P1[iy] = p + q;
        P2[iy] = p - q;
        float k = -b_ / a_;
        float l = -c / a_;
        P1[ix] = k * P1[iy] + l;
        P2[ix] = k * P2[iy] + l;

        return true;
    }
}

Solid::Solid()
{
    vertices.n = 0;
    vertices.Element = NULL;
    edges.n = 0;
    edges.Element = NULL;
    faces.n = 0;
    faces.Element = NULL;
    maxnVertexEdges = 3;
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
    solids.clear();
}

void Solid::Add(Solid *pSolid)
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
    SolidVertex *pVertexMem = vertices.Element;
    Solid *pSolid;
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
    // RVL_DELETE_ARRAY(faces.Element);
    // faces.Element = new SolidFace[faces.n];
    // SolidFace* pFaceMem = faces.Element;
    // for (iSolid = 0; iSolid < solids.size(); iSolid++)
    //{
    //     pSolid = solids[iSolid];

    //}
}

void Solid::Update()
{
    // Faces.

    ComputeFaceParams();

    // Edges.

    SolidEdge *pEdge = edges.Element;
    // int iNext;
    // int iNextVertex;
    // int iTwin;
    // int i;
    SolidEdge *pTwinEdge;
    float *P1, *P2;
    float dP[3];
    int iEdge;
    for (iEdge = 0; iEdge < edges.n; iEdge++, pEdge++)
    {
        if (iEdge < pEdge->iTwin)
        {
            P1 = vertices.Element[pEdge->iVertex].P;
            P2 = vertices.Element[edges.Element[pEdge->iNext].iVertex].P;
            RVLDIF3VECTORS(P2, P1, dP);
            pEdge->len = sqrt(RVLDOTPRODUCT3(dP, dP));
            RVLSCALE3VECTOR2(dP, pEdge->len, pEdge->V);
        }
        else
        {
            pTwinEdge = edges.Element + pEdge->iTwin;
            pEdge->len = pTwinEdge->len;
            RVLNEGVECT3(pTwinEdge->V, pEdge->V);
        }
    }
}

void Solid::Create(Array<Array<int>> faces_)
{
    if (solids.size() == 0)
    {
        // Faces.

        faces.n = faces_.n;
        faces.Element = new SolidFace[faces.n];

        // Edges.

        int iVertex;
        for (iVertex = 0; iVertex < vertices.n; iVertex++)
            vertices.Element[iVertex].iEdge = -1;
        edges.n = 0;
        int iFace;
        for (iFace = 0; iFace < faces.n; iFace++)
            edges.n += faces_.Element[iFace].n;
        edges.Element = new SolidEdge[edges.n];
        int *edgeMap = new int[vertices.n * vertices.n];
        memset(edgeMap, 0xff, vertices.n * vertices.n * sizeof(int));
        SolidEdge *pEdge;
        int iNext;
        int iEdge = 0;
        int iNextVertex;
        int iTwin;
        Array<int> *pFaceSrc = faces_.Element;
        pFaceSrc = faces_.Element;
        SolidFace *pFace = faces.Element;
        int i;
        SolidEdge *pTwinEdge;
        // float* P1, * P2;
        // float dP[3];
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
                }
                pEdge->bConvex = true;
                iEdge++;
            }
        }
        delete[] edgeMap;
        Update();
    }
}

void Solid::Move(
    Solid *pSolidSrc,
    Pose3D *pMove)
{
    // Parts.

    Solid *pPartSrc;
    Solid *pPart;
    for (int iPart = 0; iPart < solids.size(); iPart++)
    {
        pPart = solids[iPart];
        pPartSrc = pSolidSrc->solids[iPart];
        pPart->Move(pPartSrc, pMove);
    }

    // Vertices.

    SolidVertex *pVertexSrc = pSolidSrc->vertices.Element;
    SolidVertex *pVertex = vertices.Element;
    for (int iVertex = 0; iVertex < vertices.n; iVertex++, pVertex++, pVertexSrc++)
        RVLTRANSF3(pVertexSrc->P, pMove->R, pMove->t, pVertex->P);

    // Edges.
    SolidEdge *pEdgeSrc = pSolidSrc->edges.Element;
    SolidEdge *pEdge = edges.Element;
    for (int iEdge = 0; iEdge < edges.n; iEdge++, pEdge++, pEdgeSrc++)
        RVLMULMX3X3VECT(pMove->R, pEdgeSrc->V, pEdge->V);

    // Faces.

    SolidFace *pFaceSrc = pSolidSrc->faces.Element;
    SolidFace *pFace = faces.Element;
    for (int iFace = 0; iFace < faces.n; iFace++, pFace++, pFaceSrc++)
        RVLPLANETRANSF3(pFaceSrc->N, pFaceSrc->d, pMove->R, pMove->t, pFace->N, pFace->d);
}

void Solid::Copy(
    Solid *pSolidSrc,
    bool bCreate)
{
    vertices.n = pSolidSrc->vertices.n;
    if (pSolidSrc->vertices.Element != NULL && pSolidSrc->vertices.n > 0)
    {
        if (bCreate)
            vertices.Element = new SolidVertex[vertices.n];
        memcpy(vertices.Element, pSolidSrc->vertices.Element, vertices.n * sizeof(SolidVertex));
    }

    edges.n = pSolidSrc->edges.n;
    if (pSolidSrc->edges.Element != NULL && pSolidSrc->edges.n > 0)
    {
        if (bCreate)
            edges.Element = new SolidEdge[edges.n];
        memcpy(edges.Element, pSolidSrc->edges.Element, edges.n * sizeof(SolidEdge));
    }

    faces.n = pSolidSrc->faces.n;
    if (pSolidSrc->faces.Element != NULL && pSolidSrc->faces.n > 0)
    {
        if (bCreate)
            faces.Element = new SolidFace[faces.n];
        memcpy(faces.Element, pSolidSrc->faces.Element, faces.n * sizeof(SolidFace));
    }

    Solid *pPartSrc, *pPart;
    for (int iSolid = 0; iSolid < pSolidSrc->solids.size(); iSolid++)
    {
        pPartSrc = pSolidSrc->solids[iSolid];
        pPart = (bCreate ? new Solid : solids[iSolid]);
        pPart->Copy(pPartSrc, bCreate);
        if (bCreate)
            solids.push_back(pPart);
    }
}

void Solid::ComputeFaceParams()
{
    float *P1, *P2, *P3;
    float V1[3], V2[3];
    SolidFace *pFace = faces.Element;
    SolidEdge *pEdge;
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

// This function is never tried.

int Solid::GetNumVertices()
{
    int nVertices = vertices.n;
    for (int iPart = 0; iPart < solids.size(); iPart++)
        nVertices += GetNumVertices();
    return nVertices;
}

// This function is never tried.

void Solid::GetVertices(
    Array<Vector3<float>> &verticesOut,
    bool bAppend)
{
    if (!bAppend)
    {
        verticesOut.Element = new Vector3<float>[GetNumVertices()];
        verticesOut.n = 0;
    }
    float *PSrc, *PTgt;
    for (int iVertex = 0; iVertex < vertices.n; iVertex++)
    {
        PSrc = vertices.Element[iVertex].P;
        PTgt = verticesOut.Element[verticesOut.n++].Element;
        RVLCOPY3VECTOR(PSrc, PTgt);
    }
    for (int iPart = 0; iPart < solids.size(); iPart++)
        solids[iPart]->GetVertices(verticesOut, true);
}

void Solid::GetVertexEdgesAndFaces(
    int iVertex,
    Array<int> &vertexEdges,
    Array<int> &vertexFaces)
{
    vertexEdges.n = 0;
    vertexFaces.n = 0;
    int iEdge0, iEdge;
    SolidEdge *pEdge;
    iEdge = iEdge0 = vertices.Element[iVertex].iEdge;
    do
    {
        pEdge = edges.Element + iEdge;
        vertexFaces.Element[vertexFaces.n++] = pEdge->iFace;
        vertexEdges.Element[vertexEdges.n++] = iEdge;
        iEdge = edges.Element[pEdge->iPrev].iTwin;
    } while (iEdge != iEdge0);
}

void Solid::SetBoxParameters(
    float *size,
    Pose3D *pPose)
{
    float halfSize[3];
    RVLSCALE3VECTOR(size, 0.5f, halfSize);
    Vector3<float> vertices_B[8];
    Vector3<float> *pVertex_B = vertices_B;
    RVLSET3VECTOR(pVertex_B->Element, halfSize[0], halfSize[1], -halfSize[2]);
    pVertex_B++;
    RVLSET3VECTOR(pVertex_B->Element, halfSize[0], -halfSize[1], -halfSize[2]);
    pVertex_B++;
    RVLSET3VECTOR(pVertex_B->Element, -halfSize[0], -halfSize[1], -halfSize[2]);
    pVertex_B++;
    RVLSET3VECTOR(pVertex_B->Element, -halfSize[0], halfSize[1], -halfSize[2]);
    pVertex_B++;
    RVLSET3VECTOR(pVertex_B->Element, halfSize[0], halfSize[1], halfSize[2]);
    pVertex_B++;
    RVLSET3VECTOR(pVertex_B->Element, -halfSize[0], halfSize[1], halfSize[2]);
    pVertex_B++;
    RVLSET3VECTOR(pVertex_B->Element, -halfSize[0], -halfSize[1], halfSize[2]);
    pVertex_B++;
    RVLSET3VECTOR(pVertex_B->Element, halfSize[0], -halfSize[1], halfSize[2]);
    SolidVertex *pVertex = vertices.Element;
    pVertex_B = vertices_B;
    for (int iVertex = 0; iVertex < 8; iVertex++, pVertex++, pVertex_B++)
        RVLTRANSF3(pVertex_B->Element, pPose->R, pPose->t, pVertex->P);
}

Solid *Solid::CreateBox(
    float *size,
    Pose3D *pPose)
{
    // Vertices.

    vertices.n = 8;
    vertices.Element = new SolidVertex[8];
    SetBoxParameters(size, pPose);

    // Faces.

    int face[6][4] = {{0, 1, 2, 3}, {4, 5, 6, 7}, {0, 4, 7, 1}, {1, 7, 6, 2}, {2, 6, 5, 3}, {3, 5, 4, 0}};
    Array<Array<int>> faces_;
    faces_.n = 6;
    faces_.Element = new Array<int>[6];
    int *facesMem = new int[6 * 4];
    int *pFacesMem = facesMem;
    int iFace;
    for (iFace = 0; iFace < faces_.n; iFace++, pFacesMem += 4)
    {
        faces_.Element[iFace].n = 4;
        faces_.Element[iFace].Element = pFacesMem;
        memcpy(pFacesMem, face[iFace], 4 * sizeof(int));
    }

    // Create solid.

    Create(faces_);

    // Print edges.

    // SolidEdge* pEdge = edges.Element;
    // for (int iEdge = 0; iEdge < edges.n; iEdge++, pEdge++)
    //     printf("E%d: %d-%d\n", iEdge, pEdge->iVertex, RVLSOLID_EDGE_END_VERTEX_(pEdge, this));

    //

    delete[] faces_.Element;
    delete[] facesMem;

    return this;
}

bool Solid::InSolid(float *P)
{
    int iFace;
    SolidFace *pFace = faces.Element;
    for (iFace = 0; iFace < faces.n; iFace++, pFace++)
        if (RVLDOTPRODUCT3(pFace->N, P) - pFace->d > 1e-5)
            break;
    if (iFace >= faces.n)
        return true;

    for (int iPart = 0; iPart < solids.size(); iPart++)
        if (solids[iPart]->InSolid(P))
            return true;

    return false;
}

// Intersection between a line segment and a convex solid.

bool Solid::Intersect(
    float *P1,
    float *P2,
    float *sOut,
    int *iFaceOut,
    float *V,
    float &l)
{
    RVLDIF3VECTORS(P2, P1, V);
    l = sqrt(RVLDOTPRODUCT3(V, V));
    RVLSCALE3VECTOR2(V, l, V);

    int iFace;
    SolidFace *pFace;
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
                    iFaceOut[0] = iFace;
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
                iFaceOut[0] = iFace;
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
                {
                    sOut[1] = s;
                    iFaceOut[1] = iFace;
                }
                if (bIntersection[0])
                {
                    if (sOut[1] <= sOut[0])
                        return false;
                }
            }
            else
            {
                sOut[1] = s;
                iFaceOut[1] = iFace;
                bIntersection[1] = true;
            }
        }
    }

    return true;
}

bool Solid::Intersect(
    Solid *pSolid,
    Array<SolidEdgeFaceIntersection> *pIntersectionsWithOtherEdges,
    Array<SolidEdgeFaceIntersection> *pIntersectionsWithThisEdges)
{
    pIntersectionsWithOtherEdges->n = 0;
    if (pIntersectionsWithThisEdges)
        pIntersectionsWithThisEdges->n = 0;
    IntersectWithConvex(pSolid, pIntersectionsWithOtherEdges, pIntersectionsWithThisEdges);

    Array<SolidEdgeFaceIntersection> intersectionsWithOtherEdges_;
    Array<SolidEdgeFaceIntersection> intersectionsWithThisEdges_;
    Solid *pPart;
    int iIntersection;
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
            for (iIntersection = 0; iIntersection < intersectionsWithThisEdges_.n; iIntersection++)
                intersectionsWithThisEdges_.Element[iIntersection].iFacePart = iPart;
        }
        else
            IntersectWithConvex(pPart, &intersectionsWithOtherEdges_);
        for (iIntersection = 0; iIntersection < intersectionsWithOtherEdges_.n; iIntersection++)
            intersectionsWithOtherEdges_.Element[iIntersection].iEdgePart = iPart;
        pIntersectionsWithOtherEdges->n += intersectionsWithOtherEdges_.n;
        if (pIntersectionsWithThisEdges)
            pIntersectionsWithThisEdges->n += intersectionsWithThisEdges_.n;
    }

    bool bIntersection = (pIntersectionsWithOtherEdges->n > 0);
    for (iIntersection = 0; iIntersection < pIntersectionsWithOtherEdges->n; iIntersection++)
        pIntersectionsWithOtherEdges->Element[iIntersection].iFacePart = 0;
    if (pIntersectionsWithThisEdges)
    {
        bIntersection = (bIntersection || (pIntersectionsWithThisEdges->n > 0));
        for (iIntersection = 0; iIntersection < pIntersectionsWithThisEdges->n; iIntersection++)
            pIntersectionsWithThisEdges->Element[iIntersection].iEdgePart = 0;
    }

    return bIntersection;
}

bool Solid::IntersectWithConvex(
    Solid *pSolid,
    Array<SolidEdgeFaceIntersection> *pIntersectionsWithOtherEdges,
    Array<SolidEdgeFaceIntersection> *pIntersectionsWithThisEdges)
{
    // Src - solid with which the edges of Tgt are intersecting.

    Solid *pSrc = this;
    Solid *pTgt = pSolid;
    Array<SolidEdgeFaceIntersection> *pIntersections = pIntersectionsWithOtherEdges;
    int iEdge;
    SolidEdge *pEdge;
    float s[2];
    int iFace[2];
    float *P1, *P2;
    float *P1_, *P2_;
    float V[3], V_[3];
    float l;
    bool bEdgeEndPoint[2];
    SolidEdgeFaceIntersection *pIntersection;
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
            if (!pSrc->Intersect(P1, P2, s, iFace, V, l))
                continue;
            if (s[1] > 0.0f)
            {
                if (bEdgeEndPoint[1] = (s[1] >= l))
                    s[1] = l;
                if (s[0] < l)
                {
                    if (bEdgeEndPoint[0] = (s[0] < 0.0f))
                        s[0] = 0.0f;
                    P1_ = pIntersection->P[0];
                    RVLSCALE3VECTOR(V, s[0], V_);
                    RVLSUM3VECTORS(P1, V_, P1_);
                    P2_ = pIntersection->P[1];
                    RVLSCALE3VECTOR(V, s[1], V_);
                    RVLSUM3VECTORS(P1, V_, P2_);
                    RVLCOPY2VECTOR(bEdgeEndPoint, pIntersection->bEdgeEndPoint);
                    RVLCOPY2VECTOR(iFace, pIntersection->iFace);
                    pIntersection->iEdge = iEdge;
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

// #define RVLSOLID_FREEMOVE_VISUALIZE
// #define RVLSOLID_FREEMOVE_VISUALIZE_FACE_BY_FACE

float Solid::FreeMove(
    float *V,
    Solid *pObstacle,
    uchar &contactType,
    int &iThisContactFeature,
    int &iOtherContactPart,
    int &iOtherContactFeature,
    bool bVisualize)
{
#ifdef RVLSOLID_FREEMOVE_VISUALIZE
    RVLCOLORS
    if (bVisualize)
        Visualize(pVisualizer, green);
        // pVisualizer->Run();
#endif

    int nObstacleEdges = pObstacle->edges.n;
    int iSolid;
    Solid *pSolid;
    for (iSolid = 0; iSolid < pObstacle->solids.size(); iSolid++)
    {
        pSolid = pObstacle->solids[iSolid];
        nObstacleEdges += pSolid->edges.n;
    }
    nObstacleEdges /= 2;
    Array<SolidEdgeFaceIntersection> obstacleEdgeIntersections;
    obstacleEdgeIntersections.Element = new SolidEdgeFaceIntersection[nObstacleEdges];
    Array<SolidEdgeFaceIntersection> thisEdgeIntersections;
    thisEdgeIntersections.Element = new SolidEdgeFaceIntersection[edges.n * pObstacle->solids.size()];
    SolidEdgeFaceIntersection *pIntersection;
    contactType = RVLSOLID_CONTACT_TYPE_NONE;
    float tmax = sqrt(RVLDOTPRODUCT3(V, V));
    float t = tmax;
    float U[3];
    RVLSCALE3VECTOR2(V, tmax, U);
    SolidFace *pFace = faces.Element;
    Solid hull;
    int iEdge0, iEdge;
    SolidEdge *pEdge;
    SolidVertex *pVertex;
    SolidVertex *pHullVertex;
    Array<Array<int>> faces_;
    faces_.Element = new Array<int>[2 + vertices.n];
    Array<int> *pFace_;
    int *facesMem = new int[(2 + 4) * vertices.n];
    int *pFaceMem;
    int i, j;
    int n;
    float *P[2];
    float t_;
    Array<int> contactVertices;
    contactVertices.Element = new int[vertices.n];
    contactVertices.n = 0;
    bool *bContact = new bool[vertices.n];
    memset(bContact, 0, vertices.n * sizeof(bool));
    float c;
    int *iEdge_ = new int[edges.n];
    int iiEdge;
    int iVertex;
    int iPart;
    Solid *pPart;
    for (int iFace = 0; iFace < faces.n; iFace++, pFace++)
    {
        c = RVLDOTPRODUCT3(U, pFace->N);
        if (c <= 1e-6)
            continue;

        // Create a motion hull corresponding to face iFace.

        hull.vertices.Element = new SolidVertex[2 * vertices.n];
        pHullVertex = hull.vertices.Element;
        iiEdge = 0;
        iEdge0 = iEdge = pFace->iEdge;
        do
        {
            pEdge = edges.Element + iEdge;
            iEdge_[iiEdge++] = iEdge;
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
        std::vector<vtkSmartPointer<vtkActor>> actors;
        if (bVisualize)
            actors = hull.Visualize(pVisualizer, yellow);
#endif

        // Intersection of the obstacle edges and the motion hull.

        // hull.Intersect(pObstacle, &obstacleEdgeIntersections, &thisEdgeIntersections);
        hull.Intersect(pObstacle, &obstacleEdgeIntersections);

        // Determine the largest collision-free distance from the starting position to the contact with an environment edge.

#ifdef RVLSOLID_FREEMOVE_VISUALIZE
        RVLVISUALIZER_LINES_INIT(visPts, visLines, obstacleEdgeIntersections.n)
#endif

        for (i = 0; i < obstacleEdgeIntersections.n; i++)
        {
            pIntersection = obstacleEdgeIntersections.Element + i;
            P[0] = pIntersection->P[0];
            P[1] = pIntersection->P[1];
            for (j = 0; j < 2; j++)
            {
                t_ = (RVLDOTPRODUCT3(pFace->N, P[j]) - pFace->d) / c;
                if (t_ > 0.0f && t_ < t)
                {
                    if (pIntersection->bEdgeEndPoint[j])
                    {
                        t = t_;
                        contactType = RVLSOLID_CONTACT_TYPE_PLANE_POINT;
                        iThisContactFeature = iFace;
                        iOtherContactPart = pIntersection->iEdgePart;
                        pPart = pObstacle->solids[iOtherContactPart];
                        pEdge = pPart->edges.Element + pIntersection->iEdge;
                        iOtherContactFeature = (j == 0 ? pEdge->iVertex : pPart->edges.Element[pEdge->iNext].iVertex);
                    }
                    else
                    {
                        iiEdge = pIntersection->iFace[j] - 2;
                        if (iiEdge >= 0)
                        {
                            t = t_;
                            contactType = RVLSOLID_CONTACT_TYPE_EDGE_EDGE;
                            iThisContactFeature = iEdge_[iiEdge];
                            iOtherContactPart = pIntersection->iEdgePart;
                            iOtherContactFeature = pIntersection->iEdge;
                        }
                        // else
                        //{
                        //     contactType = RVLSOLID_CONTACT_TYPE_PLANE_POINT;
                        //     iThisContactFeature = iFace;
                        // }
                    }
                }
            }

#ifdef RVLSOLID_FREEMOVE_VISUALIZE
            if (bVisualize)
            {
                t_ = (RVLDOTPRODUCT3(pFace->N, P[0]) - pFace->d) / c;
                printf("t1=%f ", t_);
                t_ = (RVLDOTPRODUCT3(pFace->N, P[1]) - pFace->d) / c;
                printf("t2=%f\n", t_);
            }
            RVLCOPY3VECTOR(P[0], visPts.Element[2 * i].P);
            RVLCOPY3VECTOR(P[1], visPts.Element[2 * i + 1].P);
            visLines.Element[i].a = 2 * i;
            visLines.Element[i].b = 2 * i + 1;
#endif
        }

#ifdef RVLSOLID_FREEMOVE_VISUALIZE
        if (bVisualize)
        {
            actors.push_back(pVisualizer->DisplayLines(visPts, visLines, magenta, 2.0f));
#ifdef RVLSOLID_FREEMOVE_VISUALIZE_FACE_BY_FACE
            pVisualizer->Run();
            pVisualizer->Clear(actors);
#endif
        }
        RVLVISUALIZER_LINES_FREE(visPts, visLines)
#endif
        // Determine the largest collision-free distance from the starting position to the contact with an environment vertex.

        // for (int iPart = 0; iPart < pObstacle->solids.size(); iPart++)
        //{
        //     pPart = pObstacle->solids[iPart];
        //     pVertex = pPart->vertices.Element;
        //     for (iVertex = 0; iVertex < pPart->vertices.n; iVertex++, pVertex++)
        //     {
        //         if (hull.InSolid(pVertex->P))
        //         {
        //             t_ = (RVLDOTPRODUCT3(pFace->N, pVertex->P) - pFace->d) / c;
        //             if (t_ > 0.0f && t_ < t)
        //             {
        //                 t = t_;
        //                 contactType = RVLSOLID_CONTACT_TYPE_PLANE_POINT;
        //                 iThisContactFeature = iFace;
        //                 iOtherContactPart = iPart;
        //                 iOtherContactFeature = iVertex;
        //             }
        //         }
        //     }
        // }

        //

        hull.Clear();
    }
    delete[] hull.vertices.Element;
    delete[] faces_.Element;
    delete[] facesMem;
    delete[] obstacleEdgeIntersections.Element;
    delete[] thisEdgeIntersections.Element;
    delete[] bContact;
    delete[] iEdge_;

#ifdef RVLSOLID_FREEMOVE_VISUALIZE
#ifndef RVLSOLID_FREEMOVE_VISUALIZE_FACE_BY_FACE
    pVisualizer->Run();
    pVisualizer->Clear();
#endif
#endif

    // Check collision with obstacle faces.

    float P2[3];
    float t__[2];
    float V3Tmp[3];
    float fTmp;
    int iFace[2];
    for (int iContactVertex = 0; iContactVertex < contactVertices.n; iContactVertex++)
    {
        iVertex = contactVertices.Element[iContactVertex];
        P[0] = vertices.Element[iVertex].P;
        RVLSUM3VECTORS(P[0], V, P2);
        for (iPart = 0; iPart < pObstacle->solids.size(); iPart++)
        {
            pPart = pObstacle->solids[iPart];
            if (!pPart->Intersect(P[0], P2, t__, iFace, V3Tmp, fTmp))
                continue;
            if (t__[0] > 0.0f && t__[0] < t)
            {
                t = t__[0];
                contactType = RVLSOLID_CONTACT_TYPE_POINT_PLANE;
                iThisContactFeature = iVertex;
                iOtherContactPart = iPart;
                iOtherContactFeature = iFace[0];
            }
        }
    }

    //

    delete[] contactVertices.Element;

    return t;
}

void Solid::Log(FILE *fp)
{
    if (fp == NULL)
        return;
    for (int iSolid = 0; iSolid < solids.size(); iSolid++)
    {
        fprintf(fp, "Solid %d\n", iSolid);
        solids[iSolid]->Log(fp);
    }
    fprintf(fp, "vertices:\n");
    for (int iVertex = 0; iVertex < vertices.n; iVertex++)
    {
        for (int i = 0; i < 3; i++)
            fprintf(fp, "%f ", vertices.Element[iVertex].P[i]);
        fprintf(fp, "\n");
    }
    fprintf(fp, "faces:\n");
    for (int iFace = 0; iFace < faces.n; iFace++)
    {
        for (int i = 0; i < 3; i++)
            fprintf(fp, "%f ", faces.Element[iFace].N[i]);
        fprintf(fp, "%f\n", faces.Element[iFace].d);
    }
}

std::vector<vtkSmartPointer<vtkActor>> Solid::Visualize(
    Visualizer *pVisualizer,
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
        Point *pPt = visPts.Element;
        int iVertex;
        for (iVertex = 0; iVertex < vertices.n; iVertex++, pPt++)
            RVLCOPY3VECTOR(vertices.Element[iVertex].P, pPt->P);
        visPts.n = pPt - visPts.Element;
        Pair<int, int> *pLine = visLines.Element;
        SolidEdge *pEdge = edges.Element;
        int iEdge;
        for (iEdge = 0; iEdge < edges.n; iEdge++, pEdge++)
        {
            if (!pEdge->bConvex)
                continue;
            pLine->a = pEdge->iVertex;
            pLine->b = edges.Element[pEdge->iNext].iVertex;
            pLine++;
        }
        visLines.n = pLine - visLines.Element;
        actors.push_back(pVisualizer->DisplayLines(visPts, visLines, color));
        RVLVISUALIZER_LINES_FREE(visPts, visLines);
    }
    if (vertices.n > 0)
    {
        Array<Point> visPts;
        Point visPt;
        RVLCOPY3VECTOR(vertices.Element[4].P, visPt.P);
        visPts.n = 1;
        visPts.Element = &visPt;
        actors.push_back(pVisualizer->DisplayPointSet<float, Point>(visPts, color, 6.0f));
    }

    return actors;
}

TouchEnvModel::TouchEnvModel()
{
    model0.plane = NULL;
    model0.vertex = NULL;
    modelx.plane = NULL;
    modelx.vertex = NULL;
    verticesForUpdate.Element = NULL;
    planesForUpdate.Element = NULL;
}

TouchEnvModel::~TouchEnvModel()
{
}

void TouchEnvModel::Create(
    Solid *pSolidIn,
    int nVertices,
    int nPlanes)
{
    pSolid = pSolidIn;
    model0.vertex = new Vector3<float>[nVertices];
    model0.plane = new MOTION::Plane[nPlanes];
    modelx.vertex = new Vector3<float>[nVertices];
    modelx.plane = new MOTION::Plane[nPlanes];
    verticesForUpdate.Element = new int[nVertices];
    verticesForUpdate.n = 0;
    planesForUpdate.Element = new int[nPlanes];
    planesForUpdate.n = 0;
    bVerticesForUpdate = new bool[nVertices];
    memset(bVerticesForUpdate, 0, nVertices * sizeof(bool));
    bPlanesForUpdate = new bool[nPlanes];
    memset(bPlanesForUpdate, 0, nPlanes * sizeof(bool));
}

void TouchEnvModel::Clear()
{
    RVL_DELETE_ARRAY(model0.plane);
    RVL_DELETE_ARRAY(model0.vertex);
    RVL_DELETE_ARRAY(modelx.plane);
    RVL_DELETE_ARRAY(modelx.vertex);
    RVL_DELETE_ARRAY(verticesForUpdate.Element);
    RVL_DELETE_ARRAY(planesForUpdate.Element);
    RVL_DELETE_ARRAY(bVerticesForUpdate);
    RVL_DELETE_ARRAY(bPlanesForUpdate);
}

void MOTION::InitCircleConvex(
    QList<QLIST::Entry2<LineArc>> *pCircleConvex,
    float r,
    QLIST::Entry2<LineArc> *&pNewLineArc)
{
    pNewLineArc->data.bArc = true;
    pNewLineArc->data.P[0] = r;
    pNewLineArc->data.P[1] = 0.0f;
    pNewLineArc->data.q = 0.0f;
    RVLQLIST_ADD_ENTRY2(pCircleConvex, pNewLineArc);
    pNewLineArc++;
}

#define RVLPOSITIVE_ANGLE_DIF(q1, q2, dq) \
    {                                     \
        dq = q2 - q1;                     \
        RVLNORMANGLE(dq);                 \
        if (dq <= 0.0f)                   \
            dq += (2.0f * PI);            \
    }

void MOTION::UpdateCircleConvex(
    QList<QLIST::Entry2<LineArc>> *pCircleConvex,
    float *N,
    float d,
    float r,
    float eps,
    QLIST::Entry2<LineArc> *&pNewLineArc)
{
    float PArc[2][2];
    float q[2];
    int i;
    float dq;
    float qTop, qBottom;
    float PTop[2], PBottom[2];
    float V2Tmp[2];
    if (Line2DCircleIntersection(N[0], N[1], -d, r, PArc[0], PArc[1]))
    {
        for (i = 0; i < 2; i++)
            q[i] = atan2(PArc[i][1], PArc[i][0]);
        dq = q[1] - q[0];
        RVLNORMANGLE(dq);
        bool bFlip;
        if (bFlip = (dq < 0.0f))
            dq = -dq;
        if (dq < 2.0f * acos(1.0f - eps / r))
        {
            if (d <= 0.0f)
                RVLQLIST_INIT(pCircleConvex);
            return;
        }
        bool bCenterBelow;
        if (bCenterBelow = (d >= 0.0f))
            bFlip = !bFlip;
        if (bFlip)
        {
            float fTmp = q[0];
            q[0] = q[1];
            q[1] = fTmp;
            RVLCOPY2VECTOR(PArc[0], V2Tmp);
            RVLCOPY2VECTOR(PArc[1], PArc[0]);
            RVLCOPY2VECTOR(V2Tmp, PArc[1]);
        }
        if (bCenterBelow)
        {
            qTop = q[1] + 0.5f * dq;
            RVLNORMANGLE(qTop);
            qBottom = qTop + PI;
            RVLNORMANGLE(qBottom);
        }
        else
        {
            qBottom = q[0] + 0.5f * dq;
            RVLNORMANGLE(qBottom);
            qTop = qBottom + PI;
            RVLNORMANGLE(qTop);
        }
        PTop[0] = r * cos(qTop);
        PTop[1] = r * sin(qTop);
        PBottom[0] = -PTop[0];
        PBottom[1] = -PTop[1];
    }
    else
    {
        if (d <= 0.0f)
            RVLQLIST_INIT(pCircleConvex);
        return;
    }
    QLIST::Entry2<LineArc> *pLineArc = pCircleConvex->pFirst;
    float e;
    bool bAbove = false;
    while (pLineArc)
    {
        e = RVLDOTPRODUCT2(N, pLineArc->data.P) - d;
        if (e <= -eps)
            pLineArc->data.rel = -1;
        else if (e < eps)
            pLineArc->data.rel = 0;
        else
        {
            pLineArc->data.rel = 1;
            bAbove = true;
        }
        pLineArc = pLineArc->pNext;
    }
    QLIST::Entry2<LineArc> *pNextLineArc;
    QLIST::Entry2<LineArc> **ppNextLineArc;
    float dq_, dqTop, dqBottom;
    pLineArc = pCircleConvex->pFirst;
    while (pLineArc)
    {
        if (pLineArc->data.bArc)
        {
            pNextLineArc = (pLineArc->pNext ? pLineArc->pNext : pCircleConvex->pFirst);
            if (pLineArc->data.rel <= 0 && pNextLineArc->data.rel <= 0)
            {
                RVLPOSITIVE_ANGLE_DIF(pLineArc->data.q, pNextLineArc->data.q, dq_);
                RVLPOSITIVE_ANGLE_DIF(pLineArc->data.q, qTop, dqTop);
                if (dqTop < dq_)
                {
                    pNewLineArc->data.bArc = true;
                    pNewLineArc->data.q = qTop;
                    RVLCOPY2VECTOR(PTop, pNewLineArc->data.P);
                    pNewLineArc->data.rel = 1;
                    ppNextLineArc = &(pLineArc->pNext);
                    RVLQLIST_INSERT_ENTRY2_(ppNextLineArc, pNewLineArc);
                    pLineArc = pNewLineArc;
                    pNewLineArc++;
                    bAbove = true;
                }
            }
            if (pLineArc->data.rel >= 0 && pNextLineArc->data.rel >= 0)
            {
                RVLPOSITIVE_ANGLE_DIF(pLineArc->data.q, pNextLineArc->data.q, dq_);
                RVLPOSITIVE_ANGLE_DIF(pLineArc->data.q, qBottom, dqBottom);
                if (dqBottom < dq_)
                {
                    pNewLineArc->data.bArc = true;
                    pNewLineArc->data.q = qBottom;
                    RVLCOPY2VECTOR(PBottom, pNewLineArc->data.P);
                    pNewLineArc->data.rel = -1;
                    ppNextLineArc = &(pLineArc->pNext);
                    RVLQLIST_INSERT_ENTRY2_(ppNextLineArc, pNewLineArc);
                    pLineArc = pNewLineArc;
                    pNewLineArc++;
                }
            }
        }
        pLineArc = pLineArc->pNext;
    }
    if (!bAbove)
        return;
    QLIST::Entry2<LineArc> *pLastLineArcBelow = NULL;
    pLineArc = pCircleConvex->pFirst;
    while (pLineArc)
    {
        if (pLineArc->data.rel < 0)
        {
            pNextLineArc = (pLineArc->pNext ? pLineArc->pNext : pCircleConvex->pFirst);
            if (pNextLineArc->data.rel >= 0)
                pLastLineArcBelow = pLineArc;
        }
        pLineArc = pLineArc->pNext;
    }
    if (pLastLineArcBelow == NULL)
    {
        RVLQLIST_INIT(pCircleConvex);
        return;
    }
    QLIST::Entry2<LineArc> *pPrevLineArc = NULL;
    QLIST::Entry2<LineArc> **ppAfterCutLineArc;
    float PLastAbove[2];
    bool bLastAboveIsArc;
    float dP[2];
    float s;
    pPrevLineArc = pLastLineArcBelow;
    pLineArc = (pPrevLineArc->pNext ? pPrevLineArc->pNext : pCircleConvex->pFirst);
    QLIST::Entry2<LineArc> *pCutLineArc = NULL;
    while (true)
    {
        if (pLineArc->data.rel > 0)
        {
            RVLCOPY2VECTOR(pLineArc->data.P, PLastAbove);
            bLastAboveIsArc = pLineArc->data.bArc;
            if (pCutLineArc == NULL)
            {
                if (pLastLineArcBelow->data.bArc)
                {
                    pLineArc->data.q = q[1];
                    RVLCOPY2VECTOR(PArc[1], pLineArc->data.P);
                }
                else
                {
                    RVLDIF2VECTORS(pLineArc->data.P, pLastLineArcBelow->data.P, dP);
                    s = (d - RVLDOTPRODUCT2(N, pLastLineArcBelow->data.P)) / RVLDOTPRODUCT2(N, dP);
                    RVLSCALE2VECTOR(dP, s, dP);
                    RVLSUM2VECTORS(pLastLineArcBelow->data.P, dP, pLineArc->data.P);
                }
                pLineArc->data.bArc = false;
                pCutLineArc = pLineArc;
            }
            else
                RVLQLIST_REMOVE_ENTRY2(pCircleConvex, pLineArc, QLIST::Entry2<LineArc>);
        }
        else if (pLineArc->data.rel < 0)
        {
            if (pNewLineArc->data.bArc = bLastAboveIsArc)
            {
                pNewLineArc->data.q = q[0];
                RVLCOPY2VECTOR(PArc[0], pNewLineArc->data.P);
            }
            else
            {
                RVLDIF2VECTORS(pLineArc->data.P, PLastAbove, dP);
                s = (d - RVLDOTPRODUCT2(N, PLastAbove)) / RVLDOTPRODUCT2(N, dP);
                RVLSCALE2VECTOR(dP, s, dP);
                RVLSUM2VECTORS(PLastAbove, dP, pNewLineArc->data.P);
            }
            ppAfterCutLineArc = &(pCutLineArc->pNext);
            RVLQLIST_INSERT_ENTRY2_(ppAfterCutLineArc, pNewLineArc);
            pNewLineArc++;

            return;
        }
        else
            RVLQLIST_REMOVE_ENTRY2(pCircleConvex, pLineArc, QLIST::Entry2<LineArc>);
        pLineArc = pLineArc->pNext;
        if (pLineArc == NULL)
            pLineArc = pCircleConvex->pFirst;
    }
}

// Only for debugging purpose!!!

void MOTION::TestCircleConvex(Array<int> rndVal)
{
    // Parameters.

    float rmin = 0.8f;
    float rmax = 1.2f;
    float eps = 0.001f;
    int nSamples = 10000;
    int nCuts = 3;
    float imgScale = 200.0f;
    float kSpace = 0.2f;
    float normalSize = 0.1f;
    bool bVisualization = true;

    // Constants.

    float kRegion = rmax * (1.0f + kSpace);
    int imgCenter = (int)round(imgScale * kRegion);
    int imgSize = 2 * imgCenter + 1;
    Rect<float> region;
    region.minx = region.miny = -kRegion;
    region.maxx = region.maxy = kRegion;

    //

    auto ToImage = [](float *PSrc, float scale, float offset)
    {
        cv::Point PTgt;
        PTgt.x = (int)round(scale * (PSrc[0] + offset));
        PTgt.y = (int)round(scale * (-PSrc[1] + offset));
        return PTgt;
    };

    struct Line2D
    {
        float N[2];
        float d;
    };

    Line2D *line = new Line2D[nCuts];
    cv::Scalar black(0, 0, 0);
    cv::Scalar green(0, 255, 0);
    cv::Scalar red(0, 0, 255);
    QList<QLIST::Entry2<LineArc>> circleConvex;
    QList<QLIST::Entry2<LineArc>> *pCircleConvex = &circleConvex;
    QLIST::Entry2<LineArc> *circleConvexMem = new QLIST::Entry2<LineArc>[3 * nCuts + 1];
    QLIST::Entry2<LineArc> *pNewCircleConvex = circleConvexMem;
    int iRndVal = 0;
    float fTmp;
    float V2Tmp[2];
    float dMin;
    for (int iSample = 0; iSample < nSamples; iSample++)
    {
        float r = rmin + (rmax - rmin) * RealPseudoRand<float>(rndVal, iRndVal);
        cv::Point center(imgCenter, imgCenter);
        int ir = (int)round(r * imgScale);
        RVLQLIST_INIT(pCircleConvex);
        float Ps[2];
        Ps[0] = kRegion * (2.0f * RealPseudoRand<float>(rndVal, iRndVal) - 1.0f);
        Ps[1] = kRegion * (2.0f * RealPseudoRand<float>(rndVal, iRndVal) - 1.0f);
        pNewCircleConvex = circleConvexMem;
        MOTION::InitCircleConvex(pCircleConvex, r, pNewCircleConvex);
        for (int iCut = 0; iCut < nCuts; iCut++)
        {
            float q = PI * (1.0f - 2.0f * RealPseudoRand<float>(rndVal, iRndVal));
            line[iCut].N[0] = cos(q);
            line[iCut].N[1] = sin(q);
            dMin = RVLDOTPRODUCT2(line[iCut].N, Ps);
            line[iCut].d = dMin + (kRegion - dMin) * RealPseudoRand<float>(rndVal, iRndVal);
            // if (iSample < 17)
            //     continue;
            MOTION::UpdateCircleConvex(pCircleConvex, line[iCut].N, line[iCut].d, r, eps, pNewCircleConvex);

            if (iCut == 0)
                printf("Sample %d\n", iSample);

            // Visualization.

            if (bVisualization)
            {
                cv::Mat image(imgSize, imgSize, CV_8UC3, cv::Scalar(255, 255, 255));
                cv::circle(image, center, ir, black);
                for (int iLine = 0; iLine <= iCut; iLine++)
                {
                    float Pc[2];
                    RVLSCALE2VECTOR(line[iLine].N, line[iLine].d, Pc);
                    float V[2];
                    V[0] = -line[iLine].N[1];
                    V[1] = line[iLine].N[0];
                    fTmp = 2.0f * kRegion;
                    RVLSCALE2VECTOR(V, fTmp, V2Tmp);
                    float P[2][2];
                    RVLSUM2VECTORS(Pc, V2Tmp, P[0]);
                    RVLDIF2VECTORS(Pc, V2Tmp, P[1]);
                    float P_[2][2];
                    LineRectIntersection(P[0], P[1], region, P_[0], P_[1]);
                    float Pn[2];
                    RVLSCALE2VECTOR(line[iLine].N, normalSize, V2Tmp);
                    RVLSUM2VECTORS(Pc, V2Tmp, Pn);
                    cv::line(image, ToImage(P[0], imgScale, kRegion), ToImage(P[1], imgScale, kRegion), black);
                    cv::line(image, ToImage(Pc, imgScale, kRegion), ToImage(Pn, imgScale, kRegion), black);
                }
                QLIST::Entry2<LineArc> *pLineArc = pCircleConvex->pFirst;
                while (pLineArc)
                {
                    QLIST::Entry2<LineArc> *pNextLineArc = (pLineArc->pNext ? pLineArc->pNext : pCircleConvex->pFirst);
                    if (pLineArc->data.bArc)
                    {
                        double q1 = RAD2DEG * pLineArc->data.q;
                        double q2 = RAD2DEG * pNextLineArc->data.q;
                        if (q2 <= q1)
                            q2 += 360.0;
                        cv::ellipse(image, center, cv::Size(ir, ir), 0.0, -q1, -q2, green, 2);
                    }
                    else
                        cv::line(image, ToImage(pLineArc->data.P, imgScale, kRegion), ToImage(pNextLineArc->data.P, imgScale, kRegion), green, 2);
                    pLineArc = pLineArc->pNext;
                }
                if (pCircleConvex->pFirst)
                    cv::circle(image, ToImage(pCircleConvex->pFirst->data.P, imgScale, kRegion), 3, red, -1);
                cv::imshow("CircleConvex", image);
                cv::waitKey(0);
            }
        }
    }
    delete[] line;
}

Touch::Touch()
{
    bDoor = false;
    // kappa = 1.0f / (0.00000285f * 0.7f * 1000.0f);
    kappa = 1.0f;
    zn = 1.0f;
    stdgxyk = 0.05f;
    stdgzk = 0.02f;
    stdhxyk = 0.02f;
    stdhzk = 0.08f;
    stdphi = 3.0f; // deg
    stds = 0.01f;  // m
    stdc = 0.01f;  // m
    alpha = 1e-6;
    beta = 1e-3;
    contactAngleThr = 85.0f;
    maxReconstructionError = 0.05f; // m
    toolTiltDeg = 20.0f;            // deg
    maxOrientErrDeg = 5.0f;         // deg
    optimizationMethod = RVLMOTION_TOUCH_OPTIMIZATION_METHOD_BRUTEFORCE;
    nSamples = 1000;
    maxnAttempts = 8;
    maxnContactCombinations = 10000;
    nBest = 500;
    float xInc_[15] = {0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 1e-3, 1e-3, 1e-3, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4};
    memcpy(xInc, xInc_, 15 * sizeof(float));
    bRefPtConstraints = false;
    simulationSeed = 0;
    nSimulationTouches = 10;
    simulation = RVLMOTION_TOUCH_SIMULATION_RND;
    contactIntersectionThr = 0.1f;
    RVLUNITMX3(pose_tool_E.R);
    RVLNULL3VECTOR(pose_tool_E.t);
    bVisualization = false;
    sessionSize = 1;
    nPanels = 0;
    target.iSolid = 0;
    target.vertices.n = target.edges.n = target.faces.n = 0;
    bDebug = false;
    model_gt.pVNEnv = NULL;
    model_gt.d = NULL;
    model_gt.pEnvSolidParams = NULL;
    model_e.pVNEnv = NULL;
    model_e.d = NULL;
    model_e.pEnvSolidParams = NULL;
    model_x.pVNEnv = NULL;
    model_x.d = NULL;
    model_x.pEnvSolidParams = NULL;
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
    resultsFolder = NULL;
    bestSolutions.Element = NULL;
    contactMem = NULL;
    scenes.Element = NULL;
    GTContacts = NULL;
    targetMem = NULL;

    Constants();
}

Touch::~Touch()
{
    Clear();
}

void Touch::Create(char *cfgFileName)
{
    // Load paramters from a configuration file.

    CreateParamList();
    paramList.LoadParams(cfgFileName);

    // Load camera parameters from the configuration file.

    LoadCameraParametersFromFile(cfgFileName, camera, pMem0);

    // Constants.

    Constants();

    // Allocate memory.

    bestSolutions.Element = new MOTION::TouchSolution[nBest];
    maxnContacts = maxnAttempts * sessionSize;
    contactMem = new int[nBest * maxnContacts];
    MOTION::TouchSolution *pSolution = bestSolutions.Element;
    for (int i = 0; i < nBest; i++, pSolution++)
        pSolution->contacts = contactMem + maxnContacts * i;
    GTContacts = new int[maxnContacts];

    // Random indices.

    RVL_DELETE_ARRAY(rndVal.Element);
    rndVal.n = 1000000;
    iRndVal = 0;
    RandomIndices(rndVal);
}

void Touch::CreateParamList()
{
    paramList.m_pMem = pMem0;
    RVLPARAM_DATA *pParamData;
    paramList.Init();
    pParamData = paramList.AddParam("Touch.stdgxyk", RVLPARAM_TYPE_FLOAT, &stdgxyk);
    pParamData = paramList.AddParam("Touch.stdgzk", RVLPARAM_TYPE_FLOAT, &stdgzk);
    pParamData = paramList.AddParam("Touch.stdhxyk", RVLPARAM_TYPE_FLOAT, &stdhxyk);
    pParamData = paramList.AddParam("Touch.stdhzk", RVLPARAM_TYPE_FLOAT, &stdhzk);
    pParamData = paramList.AddParam("Touch.stdphi", RVLPARAM_TYPE_FLOAT, &stdphi);
    pParamData = paramList.AddParam("Touch.stds", RVLPARAM_TYPE_FLOAT, &stds);
    pParamData = paramList.AddParam("Touch.alpha", RVLPARAM_TYPE_FLOAT, &alpha);
    pParamData = paramList.AddParam("Touch.nSamples", RVLPARAM_TYPE_INT, &nSamples);
    pParamData = paramList.AddParam("Touch.Optimization.Method", RVLPARAM_TYPE_ID, &optimizationMethod);
    paramList.AddID(pParamData, "BF", RVLMOTION_TOUCH_OPTIMIZATION_METHOD_BRUTEFORCE);
    paramList.AddID(pParamData, "RND", RVLMOTION_TOUCH_OPTIMIZATION_METHOD_RNDSAMPLING);
    pParamData = paramList.AddParam("Touch.Simulation.seed", RVLPARAM_TYPE_INT, &simulationSeed);
    pParamData = paramList.AddParam("Touch.Simulation.num_touches", RVLPARAM_TYPE_INT, &nSimulationTouches);
    pParamData = paramList.AddParam("Touch.Simulation", RVLPARAM_TYPE_ID, &simulation);
    paramList.AddID(pParamData, "RND", RVLMOTION_TOUCH_SIMULATION_RND);
    paramList.AddID(pParamData, "OPEN", RVLMOTION_TOUCH_SIMULATION_OPEN);
    pParamData = paramList.AddParam("Touch.Simulation.session_size", RVLPARAM_TYPE_INT, &sessionSize);
    pParamData = paramList.AddParam("Touch.Visualization", RVLPARAM_TYPE_BOOL, &bVisualization);
    pParamData = paramList.AddParam("Touch.max_num_contact_combinations", RVLPARAM_TYPE_INT, &maxnContactCombinations);
    pParamData = paramList.AddParam("Touch.max_num_attempts", RVLPARAM_TYPE_INT, &maxnAttempts);
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
    RVL_DELETE_ARRAY(bestSolutions.Element);
    RVL_DELETE_ARRAY(contactMem);
    RVL_DELETE_ARRAY(scenes.Element);
    RVL_DELETE_ARRAY(GTContacts);
    RVL_DELETE_ARRAY(targetMem);
}

void Touch::LM(
    float *x0,
    Array<MOTION::TouchData> touches,
    float *x,
    MOTION::TouchLMError *pErr)
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
    double *J = (double *)(cvJ.data);
    cv::Mat cvr(touches.n, 1, CV_64FC1);
    double *r = (double *)(cvr.data);
    cv::Mat cvQ(RVLMOTION_TOUCH_NUM_PARAMS, RVLMOTION_TOUCH_NUM_PARAMS, CV_64FC1);
    double *Q = (double *)(cvQ.data);
    cv::Mat cvR(RVLMOTION_TOUCH_NUM_PARAMS, 1, CV_64FC1);
    double *R = (double *)(cvR.data);
    cv::Mat cvdx(RVLMOTION_TOUCH_NUM_PARAMS, 1, CV_64FC1);
    double *dx = (double *)(cvdx.data);
    cv::Mat cvJg(touches.n, RVLMOTION_TOUCH_NUM_PARAMS, CV_64FC1);
    double *Jg = (double *)(cvJg.data);
    cv::Mat cvg(touches.n, 1, CV_64FC1);
    double *g = (double *)(cvg.data);
    cv::Mat cvJh(refPts.n, RVLMOTION_TOUCH_NUM_PARAMS, CV_64FC1);
    double *Jh = (double *)(cvJh.data);
    cv::Mat cvh(refPts.n, 1, CV_64FC1);
    double *h = (double *)(cvh.data);
    int iRefPt;
    Array<int> ig;
    ig.Element = new int[touches.n];
    bool *bg = new bool[touches.n];
    int i;
    float xPrev[RVLMOTION_TOUCH_NUM_PARAMS];
    float oldx;
    float E, EPrev, Er, Eg, Ex, Eh, rmax, absr;
    float gmax = 0.0f;
    float ravg, Egavg; // Only for testig purpose.
    MOTION::TouchData *pTouch;
    Pair<float, float> err;
    int iConstraint;
    float dr;
    float dg;
    float dh;
    int nTouchesWithContacts; // Only for testig purpose.
    for (it = 0; it < maxnIterations; it++)
    {
        // Residuals.

        // UpdateEnvironmentModel(&envSolidParams, &model_e, x, &model_x);
        UpdatEnvSolidParams(NULL, &touches, &model_e, x, &model_x);
#ifdef RVLMOTION_TOUCH_VN
        model_x.pVNEnv->CopyDescriptor(model_x.d);
#endif
        if (pVisualizationData->bOptimization)
        {
            pVisualizationData->pVisualizer->Clear(pVisualizationData->errorActors);
            pVisualizationData->errorActors.clear();
        }
        ig.n = 0;
        nTouchesWithContacts = 0; // Only for testig purpose.
        for (iTouch = 0; iTouch < touches.n; iTouch++)
        {
            pTouch = touches.Element + iTouch;
#ifdef RVLMOTION_TOUCH_VN
            r[iTouch] = SDF(pTouch, model_x.pVNEnv, model_x.d);
#else
            if (pTouch->nContacts > 0)
            {
                nTouchesWithContacts++; // Only for testig purpose.
                err = Error(pTouch, true);
                // printf("err.a=%f err.b=%f\n", err.a, err.b);
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
        ravg = sqrt(Er / (float)nTouchesWithContacts); // Only for testig purpose.
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
        // printf("E=%f rmax=%f gmax=%f\n", E, rmax, gmax);

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
            // UpdateEnvironmentModel(&envSolidParams, &model_e, x, &model_x);
            UpdatEnvSolidParams(NULL, &touches, &model_e, x, &model_x);
#ifdef RVLMOTION_TOUCH_VN
            model_x.pVNEnv->CopyDescriptor(model_x.d);
#endif
            iConstraint = 0;
            for (iTouch = 0; iTouch < touches.n; iTouch++)
            {
                pTouch = touches.Element + iTouch;
#ifdef RVLMOTION_TOUCH_VN
                J[iTouch * RVLMOTION_TOUCH_NUM_PARAMS + i] = (SDF(pTouch, model_x.pVNEnv, model_x.d) - r[iTouch]) / xInc[i];
#else
                if (pTouch->nContacts > 0)
                {
                    err = Error(pTouch, true);
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
    // printf("Er=%f Eravg=%f Eg=%f Egavg=%f Ex=%f E=%f\n", Er, Eravg, Eg, Egavg, Ex, E);
}

float Touch::SDF(
    MOTION::TouchData *pTouchData,
    VN *pVNEnv,
    float *d)
{
    MOTION::PlanarSurface *pSurface = surfaces.Element + pTouchData->contact.iEnvFeature.a;
    RECOG::VN_::Feature *pFeature = pVNEnv->featureArray.Element + pSurface->VNFeatures.Element[0];
    float e = RVLDOTPRODUCT3(pFeature->N, pTouchData->pose.t) - pFeature->d;

    // int iActiveFeature;
    // return pVNEnv->Evaluate(pToolPose->t, SDFBuff, iActiveFeature, d);

    return e;
}

Pair<float, float> Touch::Error(
    MOTION::TouchData *pTouchData,
    bool bToolPositioned,
    bool bPointTool,
    float *pOrthogonalDist,
    float *pLateralDist)
{
    // pVisualizationData->bOptimization = (pTouchData->contact.iToolFeature == 4);       // For debugging purpose.

    RVLCOLORS
    Array<Point> visPts;
    Point visPtMem[2];
    // Array<Pair<int, int>> visLines;
    // Pair<int, int> visLine;
    if (pVisualizationData->bOptimization)
    {
        visPts.Element = visPtMem;
        visPts.n = 2;
        // visLines.Element = &visLine;
        // visLines.n = 1;
        // visLine.a = 0;
        // visLine.b = 1;
    }

    Pair<float, float> e;
    float e_;
    SolidEdge *pToolEdge;
    if (!bToolPositioned)
        toolMoved.Move(&(tool.solid), &(pTouchData->pose));
    float fTmp;
    float V3Tmp[3];
    float d;
    MOTION::Plane *pPlane, *pPlane_;
    float orthogonalDist, lateralDist;
    if (pTouchData->contact.type == RVLMOTION_TOUCH_CONTACT_TYPE_POINT_PLANE || pTouchData->contact.type == RVLMOTION_TOUCH_CONTACT_TYPE_PLANE_POINT)
    {
        float *P;
        if (pTouchData->contact.type == RVLMOTION_TOUCH_CONTACT_TYPE_POINT_PLANE)
        {
            P = pTouchData->contact.toolVertex[0];
            pPlane = pTouchData->pEnvSolidParams->modelx.plane + pTouchData->contact.iEnvPlane[0];
        }
        else
        {
            P = pTouchData->pEnvSolidParams->modelx.vertex[pTouchData->contact.iEnvVertex[0]].Element;
            pPlane = pTouchData->contact.toolPlane;
        }
        orthogonalDist = RVLDOTPRODUCT3(pPlane->N, P) - pPlane->d;
        if (pTouchData->bMiss)
        {
            e.a = 0.0f;
            e.b = (orthogonalDist > 0.0f ? 0.0f : -orthogonalDist);
            if (pLateralDist)
                *pLateralDist = 0.0f;
        }
        else
        {
            e.a = orthogonalDist;
            lateralDist = 0.0f;
            for (int i = 0; i < pTouchData->contact.nBoundaryPlanes; i++)
            {
                pPlane_ = (pTouchData->contact.type == RVLMOTION_TOUCH_CONTACT_TYPE_POINT_PLANE ? pTouchData->pEnvSolidParams->modelx.plane + contactBoundaryPlanes[pTouchData->contact.iFirstBoundaryPlane + i] : contactToolBoundaryPlanes.data() + pTouchData->contact.iFirstBoundaryPlane + i);
                e_ = RVLDOTPRODUCT3(pPlane_->N, P) - pPlane_->d;
                if (e_ > lateralDist)
                    lateralDist = e_;
            }
            e.b = lateralDist;
            if (pLateralDist)
                *pLateralDist = lateralDist;
        }

        // Visualization.

        if (pVisualizationData->bOptimization)
        {
            RVLCOPY3VECTOR(P, visPts.Element[0].P);
            float V3Tmp[3];
            RVLSCALE3VECTOR(pPlane->N, orthogonalDist, V3Tmp);
            RVLDIF3VECTORS(P, V3Tmp, visPts.Element[1].P);
            // iEdge = iEdge0;
            // do
            //{
            //     pEdge = pSolid->edges.Element + iEdge;
            //     pFace_ = pSolid->faces.Element + pSolid->edges.Element[pEdge->iTwin].iFace;
            //     e_ = RVLDOTPRODUCT3(pFace_->N, P) - pFace_->d;
            //     if (e_ > 0.0f)
            //     {
            //         fTmp = e_ + 0.001f;
            //         RVLSCALE3VECTOR(pFace_->N, fTmp, V3Tmp);
            //         RVLDIF3VECTORS(visPts.Element[1].P, V3Tmp, visPts.Element[1].P);
            //     }
            //     iEdge = pEdge->iNext;
            // } while (iEdge != iEdge0);

            pVisualizationData->envActors.push_back(pVisualizationData->pVisualizer->DisplayLine(visPts.Element, red));
        }
    }
    else if (pTouchData->contact.type == RVLMOTION_TOUCH_CONTACT_TYPE_EDGE_EDGE)
    {
        float *P11 = pTouchData->contact.toolVertex[0];
        float *P12 = pTouchData->contact.toolVertex[1];
        float *P21 = pTouchData->pEnvSolidParams->modelx.vertex[pTouchData->contact.iEnvVertex[0]].Element;
        float *P22 = pTouchData->pEnvSolidParams->modelx.vertex[pTouchData->contact.iEnvVertex[1]].Element;
        float lm1, lm2;
        float V1[3], V2[3], AC[3], BC[3], DCSqrMag, inPlaneA[3], inPlaneB[3], inPlaneBA[3];
        float P1_[3], P2_[3];
        RVL3DLINE_SEGMENTS_CLOSEST_POINTS(P11, P12, P21, P22, P1_, P2_, lm1, lm2, V1, V2, AC, BC, DCSqrMag, inPlaneA, inPlaneB, inPlaneBA, fTmp, V3Tmp);
        float edgeLen1 = sqrt(RVLDOTPRODUCT3(V1, V1));
        float U1[3];
        RVLSCALE3VECTOR2(V1, edgeLen1, U1);
        float edgeLen2 = sqrt(RVLDOTPRODUCT3(V2, V2));
        float U2[3];
        RVLSCALE3VECTOR2(V2, edgeLen2, U2);
        float N[3];
        RVLCROSSPRODUCT3(U1, U2, N);
        RVLNORM3(N, fTmp);
        if (fTmp > 1e-4)
        {
            float dP_[3];
            RVLDIF3VECTORS(P2_, P1_, dP_);
            orthogonalDist = RVLDOTPRODUCT3(dP_, N);
            RVLSCALE3VECTOR(N, orthogonalDist, V3Tmp);
            RVLDIF3VECTORS(dP_, V3Tmp, V3Tmp);
            lateralDist = sqrt(RVLDOTPRODUCT3(V3Tmp, V3Tmp));
            if (pTouchData->bMiss)
            {
                MOTION::Plane *pEdgePlane = pTouchData->contact.toolPlane;
                MOTION::Plane *pEdgePlane_ = pTouchData->contact.toolPlane + 1;
                float c[2];
                c[0] = RVLDOTPRODUCT3(pEdgePlane->N, U2);
                c[1] = RVLDOTPRODUCT3(pEdgePlane_->N, U2);
                if (RVLABS(c[1]) > RVLABS(c[0]))
                {
                    pEdgePlane = pTouchData->contact.toolPlane + 1;
                    pEdgePlane_ = pTouchData->contact.toolPlane;
                    c[0] = c[1];
                }
                float s = (pEdgePlane->d - RVLDOTPRODUCT3(pEdgePlane->N, P21)) / c[0];
                RVLSCALE3VECTOR(U2, s, V3Tmp);
                RVLSUM3VECTORS(P21, V3Tmp, V3Tmp);
                float e_ = RVLDOTPRODUCT3(pEdgePlane_->N, V3Tmp) - pEdgePlane_->d;
                e.a = 0.0f;
                e.b = (e_ >= 0.0f ? 0.0f : RVLABS(orthogonalDist));
                // if (bDebug && pTouchData->contact.iToolFeature == 4 && pTouchData->contact.iEnvFeature.a == 4 && pTouchData->contact.iEnvFeature.b == 9)
                //{
                //     RVLVISUALIZER_LINES_INIT(visPts, visLines, 1)
                //         RVLCOPY3VECTOR(P21, visPts.Element[0].P);
                //     RVLCOPY3VECTOR(P22, visPts.Element[1].P);
                //     visLines.Element[0].a = 0; visLines.Element[0].b = 1;
                //     std::vector<vtkSmartPointer<vtkActor>> debugActors;
                //     debugActors.push_back(pVisualizationData->pVisualizer->DisplayLines(visPts, visLines, magenta, 2));
                //     pVisualizationData->pVisualizer->Run();
                //     pVisualizationData->pVisualizer->Clear(debugActors);
                //     RVLVISUALIZER_LINES_FREE(visPts, visLines)
                // }
            }
            else
            {
                e.a = orthogonalDist;
                e.b = lateralDist;
            }
        }
        else
            orthogonalDist = lateralDist = e.a = e.b = 2.0f * maxReconstructionError;

        if (pLateralDist)
            *pLateralDist = lateralDist;

        // Visualization.

        if (pVisualizationData->bOptimization)
        {
            float minlm = 0.001f / edgeLen1;
            float maxlm = 1.0f - minlm;
            fTmp = RVLLIMIT(lm1, minlm, maxlm);
            RVLSCALE3VECTOR(V1, fTmp, V3Tmp);
            RVLSUM3VECTORS(P11, V3Tmp, visPts.Element[0].P);

            minlm = 0.001f / edgeLen2;
            maxlm = 1.0f - minlm;
            fTmp = RVLLIMIT(lm2, minlm, maxlm);
            RVLSCALE3VECTOR(V2, fTmp, V3Tmp);
            RVLSUM3VECTORS(P21, V3Tmp, visPts.Element[1].P);

            pVisualizationData->envActors.push_back(pVisualizationData->pVisualizer->DisplayLine(visPts.Element, red));
            // pVisualizationData->pVisualizer->Run();
        }
    }
    if (pOrthogonalDist)
        *pOrthogonalDist = orthogonalDist;

    return e;
}

#ifdef NEVER
Pair<float, float> Touch::Error(
    MOTION::TouchData *pTouchData,
    bool bToolPositioned,
    bool bPointTool,
    float *pLateralDist)
{
    // pVisualizationData->bOptimization = (pTouchData->contact.iToolFeature == 4);       // For debugging purpose.

    RVLCOLORS
    Array<Point> visPts;
    Point visPtMem[2];
    // Array<Pair<int, int>> visLines;
    // Pair<int, int> visLine;
    if (pVisualizationData->bOptimization)
    {
        visPts.Element = visPtMem;
        visPts.n = 2;
        // visLines.Element = &visLine;
        // visLines.n = 1;
        // visLine.a = 0;
        // visLine.b = 1;
    }

    Pair<float, float> e;
    float e_;
    SolidFace *pFace;
    SolidEdge *pToolEdge;
    SolidFace *pFace_;
    if (!bToolPositioned)
        toolMoved.Move(&(tool.solid), &(pTouchData->pose));
    float fTmp;
    float V3Tmp[3];
    float N[3];
    float *N_;
    float orthogonalDist, lateralDist;
    if (pTouchData->contact.type == RVLMOTION_TOUCH_CONTACT_TYPE_POINT_PLANE || pTouchData->contact.type == RVLMOTION_TOUCH_CONTACT_TYPE_PLANE_POINT)
    {
        float *P;
        Solid *pSolid;
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
        if (pTouchData->bMiss)
        {
            pToolEdge = toolMoved.edges.Element + pTouchData->contact.iToolFeature;
            float *P1 = toolMoved.vertices.Element[pToolEdge->iVertex].P;
            float *P2 = toolMoved.vertices.Element[toolMoved.edges.Element[pToolEdge->iNext].iVertex].P;
            float dP[3];
            RVLDIF3VECTORS(P2, P1, dP);
            RVLCROSSPRODUCT3(dP, pTouchData->V, N);
            RVLNORM3(N, fTmp);
            float d = RVLDOTPRODUCT3(N, P1);
            orthogonalDist = RVLDOTPRODUCT3(N, P) - d;
            e.a = 0.0f;
            e.b = (orthogonalDist > 0.0f ? 0.0f : -orthogonalDist);
            N_ = N;
            if (pLateralDist)
                *pLateralDist = 0.0f;
        }
        else
        {
            N_ = pFace->N;
            orthogonalDist = RVLDOTPRODUCT3(pFace->N, P) - pFace->d;
            e.a = orthogonalDist;
            lateralDist = 0.0f;
            int iEdge0 = pFace->iEdge;
            int iEdge = iEdge0;
            SolidEdge *pEdge;
            do
            {
                pEdge = pSolid->edges.Element + iEdge;
                pFace_ = pSolid->faces.Element + pSolid->edges.Element[pEdge->iTwin].iFace;
                e_ = RVLDOTPRODUCT3(pFace_->N, P) - pFace_->d;
                if (e_ > lateralDist)
                    lateralDist = e_;
                iEdge = pEdge->iNext;
            } while (iEdge != iEdge0);
            e.b = lateralDist;
            if (pLateralDist)
                *pLateralDist = lateralDist;
        }

        // Visualization.

        if (pVisualizationData->bOptimization)
        {
            RVLCOPY3VECTOR(P, visPts.Element[0].P);
            float V3Tmp[3];
            RVLSCALE3VECTOR(N_, orthogonalDist, V3Tmp);
            RVLDIF3VECTORS(P, V3Tmp, visPts.Element[1].P);
            // iEdge = iEdge0;
            // do
            //{
            //     pEdge = pSolid->edges.Element + iEdge;
            //     pFace_ = pSolid->faces.Element + pSolid->edges.Element[pEdge->iTwin].iFace;
            //     e_ = RVLDOTPRODUCT3(pFace_->N, P) - pFace_->d;
            //     if (e_ > 0.0f)
            //     {
            //         fTmp = e_ + 0.001f;
            //         RVLSCALE3VECTOR(pFace_->N, fTmp, V3Tmp);
            //         RVLDIF3VECTORS(visPts.Element[1].P, V3Tmp, visPts.Element[1].P);
            //     }
            //     iEdge = pEdge->iNext;
            // } while (iEdge != iEdge0);

            pVisualizationData->envActors.push_back(pVisualizationData->pVisualizer->DisplayLine(visPts.Element, red));
        }
    }
    else if (pTouchData->contact.type == RVLMOTION_TOUCH_CONTACT_TYPE_EDGE_EDGE)
    {
        float *P11, *P12;
        float P2[3];
        if (pTouchData->bMiss)
        {
            P11 = toolMoved.vertices.Element[pTouchData->contact.iToolFeature].P;
            float V[3];
            RVLSCALE3VECTOR(pTouchData->V, pTouchData->t, V);
            RVLDIF3VECTORS(P11, V, P2);
            P12 = P2;
        }
        else
        {
            pToolEdge = toolMoved.edges.Element + pTouchData->contact.iToolFeature;
            P11 = toolMoved.vertices.Element[pToolEdge->iVertex].P;
            P12 = toolMoved.vertices.Element[toolMoved.edges.Element[pToolEdge->iNext].iVertex].P;
        }
        Solid *pEnvSolid = envSolidx.solids[pTouchData->contact.iEnvFeature.a];
        SolidEdge *pEnvEdge = pEnvSolid->edges.Element + pTouchData->contact.iEnvFeature.b;
        float *P21 = pEnvSolid->vertices.Element[pEnvEdge->iVertex].P;
        float *P22 = pEnvSolid->vertices.Element[pEnvSolid->edges.Element[pEnvEdge->iNext].iVertex].P;
        float lm1, lm2;
        float V1[3], V2[3], AC[3], BC[3], DCSqrMag, inPlaneA[3], inPlaneB[3], inPlaneBA[3];
        float P1_[3], P2_[3];
        RVL3DLINE_SEGMENTS_CLOSEST_POINTS(P11, P12, P21, P22, P1_, P2_, lm1, lm2, V1, V2, AC, BC, DCSqrMag, inPlaneA, inPlaneB, inPlaneBA, fTmp, V3Tmp);
        RVLCROSSPRODUCT3(V1, V2, N);
        RVLNORM3(N, fTmp);
        if (fTmp > 1e-4)
        {
            float dP_[3];
            RVLDIF3VECTORS(P2_, P1_, dP_);
            orthogonalDist = RVLDOTPRODUCT3(dP_, N);
            RVLSCALE3VECTOR(N, orthogonalDist, V3Tmp);
            RVLDIF3VECTORS(dP_, V3Tmp, V3Tmp);
            lateralDist = sqrt(RVLDOTPRODUCT3(V3Tmp, V3Tmp));
            if (pTouchData->bMiss)
            {
                MOTION::Plane *pEdgePlane = pTouchData->contact.toolPlane;
                MOTION::Plane *pEdgePlane_ = pTouchData->contact.toolPlane + 1;
                float c = RVLDOTPRODUCT3(pEdgePlane->N, V2);
                if (RVLABS(c) < 1e-5)
                {
                    pEdgePlane = pTouchData->contact.toolPlane + 1;
                    pEdgePlane_ = pTouchData->contact.toolPlane;
                    c = RVLDOTPRODUCT3(pEdgePlane->N, V2);
                }
                float s = (pEdgePlane->d - RVLDOTPRODUCT3(pEdgePlane->N, P21)) / c;
                RVLSCALE3VECTOR(V2, s, V3Tmp);
                RVLSUM3VECTORS(P21, V3Tmp, V3Tmp);
                e.a = 0.0f;
                e.b = (RVLDOTPRODUCT3(pEdgePlane_->N, V3Tmp) - pEdgePlane_->d >= 0.0f ? 0.0f : RVLABS(orthogonalDist));
            }
            else
            {
                e.a = orthogonalDist;
                e.b = lateralDist;
            }
        }
        else
            lateralDist = e.a = e.b = 2.0f * maxReconstructionError;
        if (pLateralDist)
            *pLateralDist = lateralDist;

        // Visualization.

        if (pVisualizationData->bOptimization)
        {
            float edgeLen = sqrt(RVLDOTPRODUCT3(V1, V1));
            float minlm = 0.001f / edgeLen;
            float maxlm = 1.0f - minlm;
            fTmp = RVLLIMIT(lm1, minlm, maxlm);
            RVLSCALE3VECTOR(V1, fTmp, V3Tmp);
            RVLSUM3VECTORS(P11, V3Tmp, visPts.Element[0].P);

            edgeLen = sqrt(RVLDOTPRODUCT3(V2, V2));
            minlm = 0.001f / edgeLen;
            maxlm = 1.0f - minlm;
            fTmp = RVLLIMIT(lm2, minlm, maxlm);
            RVLSCALE3VECTOR(V2, fTmp, V3Tmp);
            RVLSUM3VECTORS(P21, V3Tmp, visPts.Element[1].P);

            pVisualizationData->envActors.push_back(pVisualizationData->pVisualizer->DisplayLine(visPts.Element, red));
            // pVisualizationData->pVisualizer->Run();
        }
    }

    return e;
}
#endif

// Move to Util.

void ThreePlanesIntersection(
    float *N1, float d1,
    float *N2, float d2,
    float *N3, float d3,
    float *P)
{
    cv::Mat cvM(3, 3, CV_32FC1);
    float *rowM = (float *)(cvM.data);
    RVLCOPY3VECTOR(N1, rowM);
    rowM += 3;
    RVLCOPY3VECTOR(N2, rowM);
    rowM += 3;
    RVLCOPY3VECTOR(N3, rowM);
    cv::Mat cvd(3, 1, CV_32FC1);
    float *d = (float *)(cvd.data);
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
    MOTION::TouchSample *pSample;
    int i;
    RECOG::VN_::Feature *pFeature;
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
        Array<Pair<RECOG::VN_::SurfaceRayIntersection, RECOG::VN_::SurfaceRayIntersection>> *pIntersection;
        float s;
        Pair<RECOG::VN_::SurfaceRayIntersection, RECOG::VN_::SurfaceRayIntersection> *pIntersectionSegment;
        int iIntersectionSegment;
        Array<Pair<float, float>> intersectionPts;
        Pair<float, float> intersectionPtMem[4];
        intersectionPts.Element = intersectionPtMem;
        Pair<float, float> *pIntersectionPt;
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
}

void Touch::DeleteTouchModel(MOTION::TouchModel *pModel)
{
    if (pModel->pVNEnv)
        delete pModel->pVNEnv;
    RVL_DELETE_ARRAY(pModel->d);
}

#define RVLSET6VECTOR(V, a, b, c, d, e, f) \
    {                                      \
        V[0] = a;                          \
        V[1] = b;                          \
        V[2] = c;                          \
        V[3] = d;                          \
        V[4] = e;                          \
        V[5] = f;                          \
    }

void Touch::CreateSceneSolid(
    float sx,
    float sy,
    float sz,
    float rx,
    float ry,
    float a,
    float b,
    float c,
    float qDeg,
    bool bUpdate)
{
    // Constants.

    float q = qDeg * DEG2RAD;

    //

    nPanels = 4;
    float scene[4][6];
    // RVLSET6VECTOR(scene[0], a, sy + 2.0f * (sx + c), sx, 0.0f, 0.0f, 0.5f * (sx + sz) + c);
    // RVLSET6VECTOR(scene[1], a, sy + 2.0f * (sx + c), sx, 0.0f, 0.0f, -(0.5f * (sx + sz) + c));
    // RVLSET6VECTOR(scene[2], a, sx, sz + 2.0f * c, 0.0f, 0.5f * (sx + sy) + c, 0.0f);
    // RVLSET6VECTOR(scene[3], a, sx, sz + 2.0f * c, 0.0f, -(0.5f * (sx + sy) + c), 0.0f);
    RVLSET6VECTOR(scene[0], a, sy, sx, 0.0f, 0.0f, 0.5f * (-sx + sz));
    RVLSET6VECTOR(scene[1], a, sy, sx, 0.0f, 0.0f, -0.5f * (-sx + sz));
    RVLSET6VECTOR(scene[2], a, sx, sz - 2.0f * sx, 0.0f, 0.5f * (-sx + sy), 0.0f);
    RVLSET6VECTOR(scene[3], a, sx, sz - 2.0f * sx, 0.0f, -(0.5f * (-sx + sy)), 0.0f);

    float *panelSize;
    int i, j;
    int iPanel;
    Solid *pPanel;
    Pose3D pose_B_W;
    RVLUNITMX3(pose_B_W.R);
    float *t_B_W;
    bool bFirstVertex = false;
    for (iPanel = 0; iPanel < nPanels; iPanel++)
    {
        panelSize = scene[iPanel];
        t_B_W = scene[iPanel] + 3;
        RVLCOPY3VECTOR(t_B_W, pose_B_W.t);
        if (bUpdate)
        {
            pPanel = envSolid.solids[iPanel];
            pPanel->SetBoxParameters(panelSize, &pose_B_W);
            pPanel->Update();
        }
        else
        {
            pPanel = new Solid;
            pPanel->CreateBox(panelSize, &pose_B_W);
            envSolid.Add(pPanel);
        }
    }
    // envSolid.Union();
    float bboxSize = BoxSize<float>(&(envSolid.bbox));

    // Door.

    if (bDoor)
    {
        Pose3D pose_B_Arot;
        RVLUNITMX3(pose_B_Arot.R);
        RVLSET3VECTOR(pose_B_Arot.t, rx, ry, 0.0f);
        // RVLSET3VECTOR(pose_B_Arot.t, 0.0f, -0.5f * sy, 0.0f);
        float cs = cos(q);
        float sn = sin(q);
        RVLROTZ(cs, sn, pose_Arot_A.R);
        RVLNULL3VECTOR(pose_Arot_A.t);
        RVLUNITMX3(doorPose.R);
        RVLSET3VECTOR(doorPose.t, -0.5f * a, 0.5f * sy, 0.0f);
        Pose3D pose_B_A;
        RVLCOMPTRANSF3D(pose_Arot_A.R, pose_Arot_A.t, pose_B_Arot.R, pose_B_Arot.t, pose_B_A.R, pose_B_A.t);
        RVLCOMPTRANSF3D(doorPose.R, doorPose.t, pose_B_A.R, pose_B_A.t, pose_B_W.R, pose_B_W.t);
        float panelSize_[3];
        RVLSET3VECTOR(panelSize_, sx, sy, sz);
        if (bUpdate)
        {
            pPanel = envSolid.solids[nPanels];
            pPanel->SetBoxParameters(panelSize_, &pose_B_W);
            pPanel->Update();
        }
        else
        {
            pPanel = new Solid;
            pPanel->CreateBox(panelSize_, &pose_B_W);
            envSolid.Add(pPanel);
        }
        nPanels++;
    }
}

#define RVLMOTION_TOUCH_NUM_COVERED_FACES 4
#define RVLMOTION_TOUCH_NUM_CONVEX_EDGES 28

void Touch::CreateScene(
    float sx,
    float sy,
    float sz,
    float rx,
    float ry,
    float a,
    float b,
    float c,
    float qDeg,
    bool bUpdate)
{
    // Scene.

    envSolid.Clear();
    CreateSceneSolid(sx, sy, sz, rx, ry, a, b, c, qDeg, bUpdate);

    // Target.

    target.iSolid = 4;
    target.vertices.n = 1;
    target.edges.n = 2;
    target.faces.n = 1;
    RVL_DELETE_ARRAY(targetMem);
    targetMem = new int[target.vertices.n + target.edges.n + target.faces.n];
    int *pTargetMem = targetMem;
    target.vertices.Element = pTargetMem;
    target.vertices.Element[0] = 7;
    pTargetMem += target.vertices.n;
    target.edges.Element = pTargetMem;
    target.edges.Element[0] = 9;
    target.edges.Element[1] = 12;
    pTargetMem += target.edges.n;
    target.faces.Element = pTargetMem;
    target.faces.Element[0] = 2;

    // Covered faces.

    int coveredFaces[RVLMOTION_TOUCH_NUM_COVERED_FACES][2] = {{2, 0}, {2, 1}, {3, 0}, {3, 1}};
    int iSolid;
    int iFace;
    Solid *pSolid;
    SolidFace *pFace;
    for (iSolid = 0; iSolid < envSolid.solids.size(); iSolid++)
    {
        pSolid = envSolid.solids[iSolid];
        pFace = pSolid->faces.Element;
        for (iFace = 0; iFace < pSolid->faces.n; iFace++, pFace++)
            pFace->bCovered = false;
    }
    for (int iCoveredFace = 0; iCoveredFace < RVLMOTION_TOUCH_NUM_COVERED_FACES; iCoveredFace++)
    {
        iSolid = coveredFaces[iCoveredFace][0];
        iFace = coveredFaces[iCoveredFace][1];
        pSolid = envSolid.solids[iSolid];
        pFace = pSolid->faces.Element + iFace;
        pFace->bCovered = true;
    }

    // Covex edges.

    int convexEdges[RVLMOTION_TOUCH_NUM_CONVEX_EDGES][3] = {{0, 0, 1}, {0, 2, 3}, {0, 4, 7}, {0, 5, 6}, {0, 6, 7}, {0, 4, 5}, {0, 0, 4}, {0, 1, 7}, {0, 2, 6}, {0, 3, 5}, {1, 0, 1}, {1, 2, 3}, {1, 4, 7}, {1, 5, 6}, {1, 1, 2}, {1, 0, 3}, {1, 0, 4}, {1, 1, 7}, {1, 2, 6}, {1, 3, 5}, {2, 0, 4}, {2, 1, 7}, {2, 2, 6}, {2, 3, 5}, {3, 0, 4}, {3, 1, 7}, {3, 2, 6}, {3, 3, 5}};
    SolidEdge *pEdge;
    int iEdge;
    for (iSolid = 0; iSolid < 4; iSolid++)
    {
        pSolid = envSolid.solids[iSolid];
        pEdge = pSolid->edges.Element;
        for (iEdge = 0; iEdge < pSolid->edges.n; iEdge++, pEdge++)
            pEdge->bConvex = false;
    }
    int iEdgeVertex[2];
    SolidVertex *pSolidVertex;
    int iEdge0;
    int i, j;
    for (int iConvexEdge = 0; iConvexEdge < RVLMOTION_TOUCH_NUM_CONVEX_EDGES; iConvexEdge++)
    {
        iSolid = convexEdges[iConvexEdge][0];
        iEdgeVertex[0] = convexEdges[iConvexEdge][1];
        iEdgeVertex[1] = convexEdges[iConvexEdge][2];
        pSolid = envSolid.solids[iSolid];
        for (i = 0; i < 2; i++)
        {
            pSolidVertex = pSolid->vertices.Element + iEdgeVertex[i];
            j = 1 - i;
            iEdge = iEdge0 = pSolidVertex->iEdge;
            do
            {
                pEdge = pSolid->edges.Element + iEdge;
                if (pSolid->edges.Element[pEdge->iNext].iVertex == iEdgeVertex[j])
                {
                    pEdge->bConvex = true;
                    break;
                }
                iEdge = pSolid->edges.Element[pEdge->iPrev].iTwin;
            } while (iEdge != iEdge0);
        }
    }

    // Visualize environment model.

    // RVLCOLORS
    // Visualizer* pVisualizer = pVisualizationData->pVisualizer;
    // if (bDoor)
    //     pVisualizer->DisplayReferenceFrame(&doorPose, 0.2f);
    // envSolid.Visualize(pVisualizer, black);
    // pVisualizer->Run();

    // Panel Surfaces.

    // Array<MOTION::Plane> panelSurfaces;
    // panelSurfaces.n = nPanels * 6;
    // panelSurfaces.Element = new MOTION::Plane[panelSurfaces.n];
    // MOTION::Plane* pPanelSurface = panelSurfaces.Element;
    // float* P_W__[3];
    // int iPlane = 0;
    // float area;
    // for (iPanel = 0; iPanel < nPanels; iPanel++)
    //{
    //     for (i = 0; i < 6; i++)
    //     {
    //         iPlane = 6 * iPanel + i;
    //         pPanelSurface = panelSurfaces.Element + iPlane;
    //         face_ = face + 2 * iPlane * 3;
    //         for(j = 0; j < 3; j++)
    //             P_W__[j] = P_W + face_[j] * 3;
    //         TriangleNormalAndArea(P_W__, pPanelSurface->N, area);
    //         pPanelSurface->d = RVLDOTPRODUCT3(pPanelSurface->N, P_W__[0]);
    //     }
    // }

    // Surfaces.

    // RVL_DELETE_ARRAY(surfaces.Element);
    // surfaces.Element = new MOTION::PlanarSurface[panelSurfaces.n];
    // MOTION::PlanarSurface *pSurface = surfaces.Element;
    // bool* bJoined = new bool[panelSurfaces.n];
    // memset(bJoined, 0, panelSurfaces.n * sizeof(bool));
    // int iPlane_;
    // MOTION::Plane* pPanelSurface_;
    // float e;
    // for (iPlane = 0; iPlane < panelSurfaces.n; iPlane++)
    //{
    //     if (bJoined[iPlane])
    //         continue;
    //     bJoined[iPlane] = true;
    //     pPanelSurface = panelSurfaces.Element + iPlane;
    //     pSurface->plane = *pPanelSurface;
    //     pSurface++;
    //     for (iPlane_ = iPlane + 1; iPlane_ < panelSurfaces.n; iPlane_++)
    //     {
    //         if (bJoined[iPlane_])
    //             continue;
    //         pPanelSurface_ = panelSurfaces.Element + iPlane_;
    //         if (1.0f - RVLDOTPRODUCT3(pPanelSurface->N, pPanelSurface_->N) > 1e-6)
    //             continue;
    //         e = pPanelSurface_->d - pPanelSurface->d;
    //         if (RVLABS(e) > 1e-6)
    //             continue;
    //         bJoined[iPlane_] = true;
    //     }
    // }
    // surfaces.n = pSurface - surfaces.Element;
    // delete[] bJoined;
    // delete[] panelSurfaces.Element;

    // VN environment model.

    if (model_gt.pVNEnv)
        delete model_gt.pVNEnv;
    model_gt.pVNEnv = new VN;
    VN *pVNEnv = model_gt.pVNEnv;
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
    VNMClusters.Element = new RECOG::VN_::ModelCluster *[VNMClusters.n];
    int iPanel;
    for (iPanel = 0; iPanel < nPanels; iPanel++)
    {
        if (bDoor && iPanel == nPanels - 1)
            VNMClusters.Element[iPanel] = pVNEnv->AddModelCluster(iPanel, RVLVN_CLUSTER_TYPE_CONVEX, pose_Arot_A.R, t, 0.5f, CT, betaInterval, NArray, pMem0);
        else
            VNMClusters.Element[iPanel] = pVNEnv->AddModelCluster(iPanel, RVLVN_CLUSTER_TYPE_CONVEX, R, t, 0.5f, CT, betaInterval, NArray, pMem0);
    }
    pVNEnv->AddOperation(nPanels, -1, 0, 1, pMem0);
    pVNEnv->AddOperation(nPanels + 1, -1, 2, 3, pMem0);
    pVNEnv->AddOperation(nPanels + 2, -1, nPanels, nPanels + 1, pMem0);
    if (bDoor)
        pVNEnv->AddOperation(nPanels + 3, -1, nPanels + 2, nPanels - 1, pMem0);
    pVNEnv->SetOutput(nPanels + 3);
    pVNEnv->Create(pMem0);
    Array<Vector3<float>> vertices_;
    vertices_.n = 0;
    int nSolidFaces = 0;
    for (iSolid = 0; iSolid < envSolid.solids.size(); iSolid++)
    {
        pSolid = envSolid.solids[iSolid];
        vertices_.n += pSolid->vertices.n;
        nSolidFaces += pSolid->faces.n;
    }
    vertices_.Element = new Vector3<float>[vertices_.n];
    Vector3<float> *pVertex_ = vertices_.Element;
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
    RECOG::VN_::Correspondence5 *pAssoc = assoc.Element;
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

    // pVisualizer = pVisualizationData->pVisualizer;
    // pVNEnv->Display(pVisualizer, 0.02f, model_gt.d);
    // pVisualizer->Run();
    // pVisualizer->Clear();

    /// Vertices.

    vertices.n = vertices_.n;
    RVL_DELETE_ARRAY(vertices.Element);
    vertices.Element = new MOTION::Vertex[vertices.n];
    MOTION::Vertex *pVertex = vertices.Element;
    RVL_DELETE_ARRAY(vertexMem);
    vertexMem = new Pair<int, int>[vertices.n];
    bool *bJoined = new bool[vertices.n];
    memset(bJoined, 0, vertices.n * sizeof(bool));
    vertices.n = 0;
    Pair<int, int> *pVertexIdx = vertexMem;
    iVertex = 0;
    int iSolid_;
    Solid *pSolid_;
    int iVertex_;
    SolidVertex *pSolidVertex_;
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
            pSolidVertex->iSrcVertex = pVertex - vertices.Element;
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
                        pSolidVertex_->iSrcVertex = pVertex - vertices.Element;
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
    MOTION::PlanarSurface *pSurface = surfaces.Element;
    RVL_DELETE_ARRAY(surfaceMem);
    surfaceMem = new uchar[pVNEnv->featureArray.n * sizeof(int) + nSolidFaces * sizeof(Pair<int, int>)];

    // Associate VN features.

    int *pFeatureIdx = (int *)surfaceMem;
    bJoined = new bool[pVNEnv->featureArray.n];
    memset(bJoined, 0, pVNEnv->featureArray.n * sizeof(bool));
    RECOG::VN_::ModelCluster *pVNCluster;
    int iFeature;
    for (iPanel = 0; iPanel < VNMClusters.n; iPanel++, pVNCluster++)
    {
        pVNCluster = VNMClusters.Element[iPanel];
        for (iFeature = pVNCluster->iFeatureInterval.a; iFeature <= pVNCluster->iFeatureInterval.b; iFeature++)
            if (iFeature - pVNCluster->iFeatureInterval.a >= 6)
                bJoined[iFeature] = true;
    }
    float e;
    int iFeature_;
    RECOG::VN_::Feature *pFeature, *pFeature_;
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

    uchar *surfaceMem_ = surfaceMem + pVNEnv->featureArray.n * sizeof(int);
    Pair<int, int> *pSolidFaceIdx = (Pair<int, int> *)surfaceMem_;
    int iSurface;
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
                        pFace->iSrcPlane = iSurface;
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

    envSolidx.Clear();
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
    Pose3D *pPose_tool_E)
{
    // Constants.

    // float xy2 = 0.5f * a * a;
    // float xy = sqrt(xy2);
    // float z = (h * h - xy2) / (2.0f * h);
    // float c = b - a;
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
    float *P;
    P = tool.solid.vertices.Element[4].P;
    RVLSET3VECTOR(P, a_, d_, h_);
    P = tool.solid.vertices.Element[5].P;
    RVLSET3VECTOR(P, a_ - c, d_, h_);
    P = tool.solid.vertices.Element[6].P;
    RVLSET3VECTOR(P, a_ - c, -d_, h_);
    P = tool.solid.vertices.Element[7].P;
    RVLSET3VECTOR(P, a_, -d_, h_);
    if (pPose_tool_E)
    {
        SolidVertex *pVertex = tool.solid.vertices.Element;
        float V3Tmp[3];
        for (int iVertex = 0; iVertex < tool.solid.vertices.n; iVertex++, pVertex++)
        {
            RVLTRANSF3(pVertex->P, pPose_tool_E->R, pPose_tool_E->t, V3Tmp);
            RVLCOPY3VECTOR(V3Tmp, pVertex->P);
        }
    }
    tool.solid.ComputeFaceParams();

    // tool.solid.vertices.n = 6;
    // tool.solid.vertices.Element = new SolidVertex[tool.solid.vertices.n];
    // SolidVertex* pVertex = tool.solid.vertices.Element;
    // RVLSET3VECTOR(pVertex->P, 0.0f, c_, h); pVertex++;
    // RVLSET3VECTOR(pVertex->P, 0.0f, -c_, h); pVertex++;
    // RVLSET3VECTOR(pVertex->P, a_, b_, 0.0f); pVertex++;
    // RVLSET3VECTOR(pVertex->P, -a_, b_, 0.0f); pVertex++;
    // RVLSET3VECTOR(pVertex->P, -a_, -b_, 0.0f); pVertex++;
    // RVLSET3VECTOR(pVertex->P, a_, -b_, 0.0f);
    // int face[5][4] = { {0, 2, 3, -1}, {0, 3, 4, 1}, {1, 4, 5, -1}, {0, 1, 5, 2}, {5, 4, 3, 2} };
    // Array<Array<int>> faces_;
    // faces_.n = 5;
    // faces_.Element = new Array<int>[5];
    // int* facesMem = new int[2 * 3 + 3 * 4];
    // int maxnFaceVertices = 4;
    // int* pFacesMem = facesMem;
    // int iFace;
    // int i;
    // Array<int> *pFace_;
    // for (iFace = 0; iFace < faces_.n; iFace++)
    //{
    //     pFace_ = faces_.Element + iFace;
    //     pFace_->Element = pFacesMem;
    //     for (i = 0; i < maxnFaceVertices; i++)
    //         if (face[iFace][i] >= 0)
    //             *(pFacesMem++) = face[iFace][i];
    //     pFace_->n = pFacesMem - pFace_->Element;
    // }
    // tool.solid.Create(faces_);

    // Visualize solid.

    // Visualizer* pVisualizer = pVisualizationData->pVisualizer;
    // tool.solid.Visualize(pVisualizer);
    // pVisualizer->Run();

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
    float *N = NArray.Element;
    SolidFace *pFace;
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

    // Box<float> bbox;
    // P = vertices.Element;
    // InitBoundingBox<float>(&bbox, P);
    // P += 3;
    // for (int i = 1; i < vertices.h; i++, P += 3)
    //     UpdateBoundingBox<float>(&bbox, P);
    // Visualizer* pVisualizer = pVisualizationData->pVisualizer;
    // tool.pVN->Display(pVisualizer, 0.02f, tool.d, NULL, 0.0f, &bbox);
    // pVisualizer->Run();
    // pVisualizer->Clear();

    // Edges.

    //

    delete[] NArray.Element;
}

// #define RVLMOTION_TOUCH_SIMULATION_SAMPLING
#define RVLMOTION_TOUCH_OUTCOME_SUCCESS 0
#define RVLMOTION_TOUCH_OUTCOME_COLLISION 1
#define RVLMOTION_TOUCH_OUTCOME_MISS 2
#define RVLMOTION_TOUCH_OUTCOME_FAILURE 3
#define RVLMOTION_TOUCH_OUTCOME_NO_TOUCH 4

void Touch::Simulation(std::vector<MOTION::DoorExperimentParams> &simParams)
{
    MOTION::DoorExperimentParams *pSimParams = simParams.data();

    // Test CircleConvex.

    // MOTION::TestCircleConvex(rndVal);

    // Parameters.

    float cameraDistance = 1.0f; // m
    float cameraHeight = 0.5f;   // m

    // Constants.

    // For visualization.

    RVLCOLORS
    Visualizer *pVisualizer = pVisualizationData->pVisualizer;
    bool bVisualizeOptimization = pVisualizationData->bOptimization;
    bool bVisualizeContacts = pVisualizationData->bContacts;
    if (!bVisualization)
    {
        pVisualizationData->bOptimization = false;
        pVisualizationData->bContacts = false;
    }

    //

    if (simulationSeed >= 0)
        iRndVal = simulationSeed;

    // Create scene.

    CreateScene(pSimParams->sx, pSimParams->sy, pSimParams->sz, pSimParams->rx, pSimParams->ry,
                pSimParams->a, pSimParams->b, pSimParams->c, pSimParams->qDeg);

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
    float *X_C_W = R_W_C;
    float *Y_C_W = R_W_C + 3;
    float *Z_C_W = R_W_C + 6;
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
    MOTION::TouchEnvModel envSolidParams;
    envSolidParams.Create(&envSolid, vertices.n, surfaces.n);
    model_e.pEnvSolidParams = &(envSolidParams.model0);
    SimulateVisionWithError(x);

    // Test model correction.

    // UpdateEnvironmentModel(&model_e, x, &model_x);
    //{
    //     int iFeature;
    //     RECOG::VN_::Feature* pFeatureGT, * pFeaturex;
    //     MOTION::Plane plane_W;
    //     RVLINVTRANSF3D(pose_W_E.R, pose_W_E.t, pose_E_W.R, pose_E_W.t);
    //     float eN, ed;
    //     for (iFeature = 0; iFeature < model_gt.pVNEnv->featureArray.n; iFeature++)
    //     {
    //         pFeatureGT = model_gt.pVNEnv->featureArray.Element + iFeature;
    //         pFeaturex = model_x.pVNEnv->featureArray.Element + iFeature;
    //         RVLPLANETRANSF3(pFeaturex->N, pFeaturex->d, pose_E_W.R, pose_E_W.t, plane_W.N, plane_W.d);
    //         RVLDIF3VECTORS(pFeatureGT->N, plane_W.N, V3Tmp);
    //         eN = sqrt(RVLDOTPRODUCT3(V3Tmp, V3Tmp));
    //         ed = pFeatureGT->d - plane_W.d;
    //         printf("eN=%f ed=%f\n", eN, ed);
    //     }
    // }

    float x_gt[RVLMOTION_TOUCH_NUM_PARAMS];
    memcpy(x_gt, x, RVLMOTION_TOUCH_NUM_PARAMS * sizeof(float));
    float E_gt = 0.0f;
    int i;
    for (i = 0; i < RVLMOTION_TOUCH_NUM_PARAMS; i++)
        E_gt += x_gt[i] * x_gt[i] / varx[i];
    E_gt *= alpha;
    printf("E_gt=%f\n", E_gt);
    memset(x, 0, RVLMOTION_TOUCH_NUM_PARAMS * sizeof(float));
    // UpdateEnvironmentModel(&model_e, x, &model_x);

    /// Sample points on the surface of the target object reconstructed by the simulated vision system.

    // model_x.pVNEnv->CopyDescriptor(model_x.d);
    // model_gt.pVNEnv->CopyDescriptor(model_gt.d);

    // Parameters.

    float rayDist = 0.01f;
    float approachDist = 0.3f;

    //

    Array<MOTION::TouchSample> samples;

#ifdef RVLMOTION_TOUCH_SIMULATION_SAMPLING
    Sample(nSamples, rayDist, samples);
#else
    samples.n = nSamples;
    samples.Element = new MOTION::TouchSample[1];
    MOTION::TouchSample *pSample = samples.Element;
    Solid *pSolid = envSolidx.solids[3];
    float P_W_[4][3];
    int iRefVertex[] = {2, 6, 1, 3};
    float *P_E_;
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

    // Sample(10000, rayDist, samples);
    //{
    //     model_gt.pVNEnv->Display(pVisualizer, 0.02f, model_gt.d);
    //     Array<Point> visPts;
    //     visPts.Element = new Point[samples.n];
    //     visPts.n = samples.n;
    //     for (int iSample = 0; iSample < samples.n; iSample++)
    //     {
    //         RVLCOPY3VECTOR(samples.Element[iSample].P, visPts.Element[iSample].P);
    //     }
    //     pVisualizer->DisplayPointSet<float, Point>(visPts, darkGreen, 4.0f);
    //     pVisualizer->Run();
    // }

    /// Touches.

    // Create tool for touches.

    toolMoved.Clear();
    toolMoved.Copy(&(tool.solid));

    toolMoved.pVisualizer = pVisualizer;

    // envSolid_E <- envSolid transformed to the r.f. E

    envSolid_E.Clear();
    envSolid_E.Copy(&envSolid);
    envSolid_E.Move(&envSolid, &pose_W_E);

    // Visualize true environment model transformed to the r.f. E.

    if (bVisualization)
        envSolid_E.Visualize(pVisualizer, black);
        // pVisualizer->Run();

        // For each touch: move tool to the sample point or to the contact with an obstacle.

        // Array<Pose3D> touches_W;
        // touches_W.Element = new Pose3D[nSamples];
        // touches_W.n = 0;
        // Pose3D* pTouch_W;
        // float P_W[3], P1_W[3], P2_W[3];
        // Array<Pair<RECOG::VN_::SurfaceRayIntersection, RECOG::VN_::SurfaceRayIntersection>>* pIntersection;
        // Pair<RECOG::VN_::SurfaceRayIntersection, RECOG::VN_::SurfaceRayIntersection>* pIntersectionSegment;
        // int iIntersectionSegment;
        // int iSample;
        // float moveDist;
        // float t;
        // float R_W_G[9];
        // float* X_G_W = R_W_G;
        // float* Y_G_W = R_W_G + 3;
        // float* Z_G_W = R_W_G + 6;
        // float move[3];
        // float P2_E[3];
        // float moveDist_ = 2.0f * approachDist;
        // MOTION::TouchData touch_E;
        // for (iSample = 0; iSample < nSamples; iSample++)
        //{
        //     pSample = samples.Element + iSample;
        //     moveDist = pSample->direction * approachDist;
        //     RVLCOPY3VECTOR(pSample->P, P1_W);
        //     RVLCOPY3VECTOR(pSample->P, P2_W);
        //     P1_W[pSample->iAxis] = pSample->P[pSample->iAxis] - moveDist;
        //     P2_W[pSample->iAxis] = pSample->P[pSample->iAxis] + moveDist;
        //     pTouch_W = touches_W.Element + touches_W.n;
        //     RVLNULLMX3X3(R_W_G);
        //     Z_G_W[pSample->iAxis] = pSample->direction;
        //     X_G_W[(pSample->iAxis + 1) % 3] = 1.0f;
        //     RVLCROSSPRODUCT3(Z_G_W, X_G_W, Y_G_W);
        //     RVLCOPYMX3X3T(R_W_G, pTouch_W->R);
        //     RVLCOPY3VECTOR(P1_W, pTouch_W->t);
        //     touches_W.n++;
        //     RVLCOMPTRANSF3D(pose_W_E.R, pose_W_E.t, pTouch_W->R, pTouch_W->t, touch_E.pose.R, touch_E.pose.t);
        //     toolMoved.Move(&(tool.solid), &(touch_E.pose));
        //     RVLTRANSF3(P2_W, pose_W_E.R, pose_W_E.t, P2_E);
        //     RVLDIF3VECTORS(P2_E, touch_E.pose.t, move);
        //     t = toolMoved.FreeMove(move, &envSolid_E);
        //     fTmp = t / moveDist_;
        //     RVLSCALE3VECTOR(move, fTmp, V);
        //     RVLSUM3VECTORS(touch_E.pose.t, V, touch_E.pose.t);
        //     touch_E.contact.iToolFeature = 0;
        //     touch_E.contact.iEnvFeature.a = 3;
        //     //touch_E.iEnvFeature.b = 12;
        //     //touch_E.type = RVLMOTION_TOUCH_CONTACT_TYPE_EDGE_EDGE;
        //     touch_E.contact.iEnvFeature.b = 2;
        //     touch_E.contact.type = RVLMOTION_TOUCH_CONTACT_TYPE_POINT_PLANE;
        //     touches_E.push_back(touch_E);

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

        // int iActiveFeature;
        // int iTouch;
        // Pose3D* pTouch;
        // float e;
        // float emin;
        // float emax;
        // float eavg;
        // float P_E[3];
        // for (iTouch = 0; iTouch < touches_W.n; iTouch++)
        //{
        //     pTouch = touches_W.Element + iTouch;
        //     RVLTRANSF3(pTouch->t, pose_W_E.R, pose_W_E.t, P_E);
        //     e = model_x.pVNEnv->Evaluate(P_E, SDFBuff, iActiveFeature, true, model_x.d);
        //     if (iTouch == 0)
        //         emin = emax = eavg = e;
        //     else
        //     {
        //         if (e < emin)
        //             emin = e;
        //         else if (e > emax)
        //             emax = e;
        //         eavg += e;
        //     }
        // }
        // eavg /= (float)(touches_W.n);
        // printf("emin=%f emax=%f eavg=%f\n", emin, emax, eavg);

#ifdef RVLMOTION_TOUCH_SIMULATION_SAMPLING
    delete[] samples.Element;
#endif

    /// Find the optimal model.

    // Transform touches to RF E.
    // Associate touches with surfaces.

    // int iTouch;
    // float Z_Ek_E[3];
    // float e;
    // float mine;
    // RECOG::VN_::Feature* pFeature;
    // for (iTouch = 0; iTouch < touches_W.n; iTouch++)
    //{
    //     pTouch_W = touches_W.Element + iTouch;
    //     pTouch_E = touches_E.Element + iTouch;
    //     RVLCOMPTRANSF3D(pose_W_E.R, pose_W_E.t, pTouch_W->R, pTouch_W->t, pTouch_E->pose.R, pTouch_E->pose.t);
    //     RVLCOPYCOLMX3X3(pTouch_E->pose.R, 2, Z_Ek_E);
    //     mine = -1.0f;
    //     pTouch_E->iEnvFeature.a = -1;
    //     for (iSurface = 0; iSurface < surfaces.n; iSurface++)
    //     {
    //         pSurface = surfaces.Element + iSurface;
    //         pFeature = model_e.pVNEnv->featureArray.Element + pSurface->VNFeatures.Element[0];
    //         if (RVLDOTPRODUCT3(Z_Ek_E, pFeature->N) > -COS45)
    //             continue;
    //         e = RVLDOTPRODUCT3(pFeature->N, pTouch_E->pose.t) - pFeature->d;
    //         e = RVLABS(e);
    //         if (mine < 0.0f || e < mine)
    //         {
    //             mine = e;
    //             pTouch_E->iEnvFeature.a = iSurface;
    //         }
    //     }

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
    // pVisualizer->Run();
    pVisualizer->Clear();

    /// Touch and correct.

    FILE *fpLog = NULL;
    if (resultsFolder)
    {
        fpLog = fopen((std::string(resultsFolder) + RVLFILEPATH_SEPARATOR + "touch_success.log").data(), "w");
        fclose(fpLog);
    }

    if (bRefPtConstraints)
        RefPts();

    Box<float> bbox;
    std::vector<MOTION::TouchData> touches_E;
    Pose3D pose_Ek_E;
    Array<Vector3<float>> path;
    path.Element = new Vector3<float>[2];
    MOTION::TouchPoint touchPt;
    float xOpt[RVLMOTION_TOUCH_NUM_PARAMS];
    Array<MOTION::TouchData> touches;
    touches.n = 0;
    std::vector<MOTION::Contact> contacts;
    MOTION::TouchData touch;
    int iLastSegment;
    float PTouch[3];
    float V[3];
    Pose3D pose_Ek_W_gt;
    int iSimulation = 0;
    int iTask = 0;
    int iAttempt;
    uchar outcome;
    std::string strOutcome;
    bestSolutions.n = 0;
    float t;
    ClearSession(touches_E, contacts);
    contactBoundaryPlanes.reserve(10000);
    contactToolBoundaryPlanes.reserve(10000);
    MOTION::TouchEnvModel *pEnvSolidParams;
    MOTION::Contact contactGT;
    float h;
    while (simulation == RVLMOTION_TOUCH_SIMULATION_RND && touches_E.size() < nSimulationTouches ||
           simulation == RVLMOTION_TOUCH_SIMULATION_OPEN && iSimulation < simParams.size())
    {
        if (resultsFolder)
            fpLog = fopen((std::string(resultsFolder) + RVLFILEPATH_SEPARATOR + "touch_success.log").data(), "a");

        touchPt.iPanel = touchPt.iFace = -1;
        // if (touches_E.size() == 0)
        //     touchPt.iPanel = 3;
        if (simulation == RVLMOTION_TOUCH_SIMULATION_RND)
            RndTouchPoint(touchPt);
        else if (simulation == RVLMOTION_TOUCH_SIMULATION_OPEN)
        {
            if (iTask == 0)
            {
                printf("\n==========================\n\n");
                printf("New session\n\n");
                printf("==========================\n\n");
            }
            printf("Task %d\n", iTask);

            // pSimParams <- simulation parameters from simParams

            pSimParams = simParams.data() + iSimulation;
            printf("\nSimulation %d\n\n", pSimParams->idx);

            // Create scene.

            CreateSceneSolid(pSimParams->sx, pSimParams->sy, pSimParams->sz, pSimParams->rx, pSimParams->ry,
                             pSimParams->a, pSimParams->b, pSimParams->c, pSimParams->qDeg, true);
            CopyVerticesAndPlanesFromSolid();

            // Flange pose from which the scene is captured.

            Pose3D pose_0_A;
            RVLINVTRANSF3D(pSimParams->pose_A_0_gt.R, pSimParams->pose_A_0_gt.t, pose_0_A.R, pose_0_A.t);
            Pose3D pose_0_W;
            RVLCOMPTRANSF3D(doorPose.R, doorPose.t, pose_0_A.R, pose_0_A.t, pose_0_W.R, pose_0_W.t);

            fTmp = sqrt(RVLDOTPRODUCT2(pose_0_W.t, pose_0_W.t));
            RVLSCALE2VECTOR2(pose_0_W.t, fTmp, V);
            h = cameraHeight + pose_0_W.t[2];
            fTmp = sqrt(cameraDistance * cameraDistance - h * h);
            RVLSCALE2VECTOR(V, fTmp, pose_E_W.t);
            pose_E_W.t[2] = h;
            fTmp = sqrt(RVLDOTPRODUCT3(pose_E_W.t, pose_E_W.t));
            float R_W_E[9];
            float *X_E_W = R_W_E;
            float *Y_E_W = R_W_E + 3;
            float *Z_E_W = R_W_E + 6;
            RVLSCALE3VECTOR2(pose_E_W.t, fTmp, Z_E_W);
            float Z[3];
            RVLSET3VECTOR(Z, 0.0f, 0.0f, 1.0f);
            RVLCROSSPRODUCT3(Z_E_W, Z, X_E_W);
            RVLNORM3(X_E_W, fTmp);
            RVLCROSSPRODUCT3(Z_E_W, X_E_W, Y_E_W);
            RVLCOPYMX3X3T(R_W_E, pose_E_W.R);
            RVLINVTRANSF3D(pose_E_W.R, pose_E_W.t, pose_W_E.R, pose_W_E.t);

            // Allocate memory for a new scene.

            if (scenes.Element == NULL)
                scenes.Element = new MOTION::TouchEnvModel[sessionSize];
            pEnvSolidParams = scenes.Element + scenes.n;
            pEnvSolidParams->idx = scenes.n;
            scenes.n++;
            pEnvSolidParams->Create(&envSolid, vertices.n, surfaces.n);

            // Associate model_e and model_x with the new scene.

            model_e.pEnvSolidParams = &(pEnvSolidParams->model0);
            model_x.pEnvSolidParams = &(pEnvSolidParams->modelx);

            // Transform scene from r.f. W to r.f. E.

            envSolid_E.Move(&envSolid, &pose_W_E);

            // if (bDebug = (pSimParams->idx == 33))
            //{
            //     bVisualization = true;
            //     pVisualizationData->bContacts = true;
            // }
            // else
            //{
            //     bVisualization = false;
            //     pVisualizationData->bOptimization = false;
            //     pVisualizationData->bContacts = false;
            // }

            // Simulate vision with error: model_e <- scene model created by simulated vision system.

            model_gt.pose_A_E = doorPose;
            if (iTask == 0)
            {
                // if (simulationSeed >= 0)
                //     iRndVal = simulationSeed;
                SimulateVisionWithError(x_gt);
            }
            else
                SimulateVisionWithError(x_gt, false);

            // model_x <- scene model obtained by correcting model_e with correction x

            UpdateEnvironmentModel(pEnvSolidParams, &model_e, x, &model_x);
            UpdateDoorOrientation(&model_x);
            SceneBBox(&(pEnvSolidParams->model0), &bbox);

            // if (bVisualization)
            //{
            //     envSolid_E.Visualize(pVisualizer, black);
            //     pVisualizationData->envActors = envSolidx.Visualize(pVisualizationData->pVisualizer, green);
            //     pVisualizationData->envActors.push_back(pVisualizer->DisplayReferenceFrame(&(model_x.pose_A_E), 0.2f));
            // }
            printf("GT:\n");
            PrintX(x_gt);
            E_gt = 0.0f;
            for (i = 0; i < 12; i++)
                E_gt += x_gt[i] * x_gt[i] / varx[i];
            E_gt *= alpha;
            printf("E_gt=%f*1e-6\n", 1e6 * E_gt);

            // pose_Ek_W_gt <- tool pose from pSimParams

            RVLCOMPTRANSF3D(pose_0_W.R, pose_0_W.t, pSimParams->pose_E_0.R, pSimParams->pose_E_0.t, pose_Ek_W_gt.R, pose_Ek_W_gt.t);
            // toolMoved.Move(&(tool.solid), &pose_Ek_E);
            // toolMoved.Visualize(pVisualizer, yellow);
            // pVisualizer->Run();
            // pVisualizer->Clear();
            // break;
        }
        // int debug = 20;
        // if (pSimParams->idx != debug)
        //{
        //     if (bVisualization)
        //         pVisualizer->Clear();
        //     if (fpLog)
        //         fclose(fpLog);
        //     iSimulation++;
        //     iTask++;
        //     if (pSimParams->idx < debug)
        //         continue;
        //     else if (pSimParams->idx > debug)
        //         break;
        // }
        iAttempt = 0;
        do
        {
            iAttempt++;
            printf("\nAttempt %d\n", iAttempt);
            if (simulation == RVLMOTION_TOUCH_SIMULATION_RND)
            {
                PlanTouch(touchPt, pose_Ek_E, path, PTouch);
                if (bVisualization)
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
            }
            else if (simulation == RVLMOTION_TOUCH_SIMULATION_OPEN)
            {
                Pose3D pose_D_E;
                OpenDoorContactPoint(pose_Ek_W_gt, pose_E_W, pose_Ek_E, pose_D_E);
                float zMoveLen = 0.4f;
                RVLCOPYCOLMX3X3(pose_Ek_E.R, 2, V);
                RVLSCALE3VECTOR(V, zMoveLen, V);
                RVLCOPY3VECTOR(pose_Ek_E.t, path.Element[0].Element);
                RVLDIF3VECTORS(pose_Ek_E.t, V, pose_Ek_E.t);
                RVLCOPYCOLMX3X3(pose_D_E.R, 2, V);
                RVLSCALE3VECTOR(V, zMoveLen, V);
                RVLDIF3VECTORS(path.Element[0].Element, V, path.Element[1].Element);
                path.n = 2;
                // if (bVisualization)
                //{
                //     Pose3D poseDebug = pose_Ek_E;
                //     RVLCOPY3VECTOR(path.Element[0].Element, poseDebug.t);
                //     toolMoved.Move(&(tool.solid), &poseDebug);
                //     //toolMoved.Visualize(pVisualizer, white);
                //     RVLDIF3VECTORS(poseDebug.t, pose_Ek_E.t, V);
                //     RVLVISUALIZER_LINES_INIT(visPts, visLines, toolMoved.vertices.n)
                //     for (int i = 0; i < toolMoved.vertices.n; i++)
                //     {
                //         RVLCOPY3VECTOR(toolMoved.vertices.Element[i].P, visPts.Element[2 * i].P);
                //         RVLDIF3VECTORS(toolMoved.vertices.Element[i].P, V, visPts.Element[2 * i + 1].P);
                //         visLines.Element[i].a = 2 * i;
                //         visLines.Element[i].b = 2 * i + 1;
                //     }
                //     pVisualizer->DisplayLines(visPts, visLines, white);
                //     RVLVISUALIZER_LINES_FREE(visPts, visLines)
                //     RVLSCALE3VECTOR(V, 0.922f, V);
                //     RVLSUM3VECTORS(pose_Ek_E.t, V, poseDebug.t);
                //     toolMoved.Move(&(tool.solid), &poseDebug);
                //     toolMoved.Visualize(pVisualizer, white);
                //     //toolMoved.Move(&(tool.solid), &pose_Ek_E);
                //     //toolMoved.Visualize(pVisualizer, white);
                // }
            }
            SimulateMove(pose_Ek_E, path, pose_Ek_E, V, t, iLastSegment, &contactGT);
            switch (iLastSegment)
            {
            case 0:
                outcome = RVLMOTION_TOUCH_OUTCOME_COLLISION;
                break;
            case 1:
                outcome = RVLMOTION_TOUCH_OUTCOME_COLLISION;
                switch (contactGT.type)
                {
                case RVLMOTION_TOUCH_CONTACT_TYPE_POINT_PLANE:
                    for (i = 0; i < target.faces.n; i++)
                        if (contactGT.iEnvFeature.a == target.iSolid && contactGT.iEnvFeature.b == target.faces.Element[i])
                        {
                            outcome = RVLMOTION_TOUCH_OUTCOME_SUCCESS;
                            break;
                        }
                case RVLMOTION_TOUCH_CONTACT_TYPE_EDGE_EDGE:
                    for (i = 0; i < target.edges.n; i++)
                        if (contactGT.iEnvFeature.a == target.iSolid && contactGT.iEnvFeature.b == target.edges.Element[i])
                        {
                            outcome = RVLMOTION_TOUCH_OUTCOME_SUCCESS;
                            break;
                        }
                case RVLMOTION_TOUCH_CONTACT_TYPE_PLANE_POINT:
                    for (i = 0; i < target.vertices.n; i++)
                        if (contactGT.iEnvFeature.a == target.iSolid && contactGT.iEnvFeature.b == target.vertices.Element[i])
                        {
                            outcome = RVLMOTION_TOUCH_OUTCOME_SUCCESS;
                            break;
                        }
                }
                if (outcome == RVLMOTION_TOUCH_OUTCOME_COLLISION)
                    int debug = 0;
                break;
            case 2:
                outcome = RVLMOTION_TOUCH_OUTCOME_MISS;
                strOutcome = "Miss";
            }
            if (outcome == RVLMOTION_TOUCH_OUTCOME_SUCCESS)
                printf("Success\n");
            if (bVisualization)
            {
                envSolid_E.Visualize(pVisualizer, black);
                pVisualizationData->envActors = envSolidx.Visualize(pVisualizationData->pVisualizer, green);
                pVisualizationData->envActors.push_back(pVisualizer->DisplayReferenceFrame(&(model_x.pose_A_E), 0.2f));
                toolMoved.Move(&(tool.solid), &pose_Ek_E);
                toolMoved.Visualize(pVisualizer, yellow);
                pVisualizer->Run();
                pVisualizationData->pVisualizer->Clear(pVisualizationData->envActors);
            }
            // if (outcome != RVLMOTION_TOUCH_OUTCOME_SUCCESS && iAttempt < maxnAttempts)
            if (outcome != RVLMOTION_TOUCH_OUTCOME_SUCCESS)
            {
                touch.pose = pose_Ek_E;
                RVLCOPY3VECTOR(V, touch.V);
                touch.t = t;
                touch.bMiss = (outcome == RVLMOTION_TOUCH_OUTCOME_MISS);
                touch.iFirstContact = -1;
                touch.pEnvSolidParams = pEnvSolidParams;
                touches_E.push_back(touch);

                // Correct camera-robot model using information obtained by touches.

                touches.n = touches_E.size();
                touches.Element = touches_E.data();
                // if (pSimParams->idx == 16)
                //{
                //     FILE* fpDebug = fopen("C:\\RVL\\Debug\\envSolid.txt", "w");
                //     envSolid.Log(fpDebug);
                //     fclose(fpDebug);
                //     fpDebug = fopen("C:\\RVL\\Debug\\model_e.txt", "w");
                //     for (int i = 0; i < surfaces.n; i++)
                //     {
                //         for (int j = 0; j < 3; j++)
                //             fprintf(fpDebug, "%f ", model_e.plane[i].N[j]);
                //         fprintf(fpDebug, "%f\n", model_e.plane[i].d);
                //     }
                //     fclose(fpDebug);
                //     fpDebug = fopen("C:\\RVL\\Debug\\x.txt", "w");
                //     for (int i = 0; i < 12; i++)
                //         fprintf(fpDebug, "%f ", x_gt[i]);
                //     fclose(fpDebug);
                // }
                memset(x, 0, RVLMOTION_TOUCH_NUM_PARAMS * sizeof(float));
                // if (!Correction(x, touches, contacts, xOpt, &contactGT, x_gt))
                if (!Correction(x, touches, contacts, xOpt))
                    break;

                // Visualize the corrected model.

                UpdateEnvironmentModel(pEnvSolidParams, &model_e, xOpt, &model_x);
                UpdateDoorOrientation(&model_x);
                UpdateEnvironmentVNModel(&model_e, xOpt, &model_x);
                if (bVisualization)
                {
                    pVisualizationData->envActors = envSolidx.Visualize(pVisualizationData->pVisualizer, green);
                    pVisualizationData->envActors.push_back(pVisualizer->DisplayReferenceFrame(&(model_x.pose_A_E), 0.2f));
                    // pVisualizationData->envActors.push_back(model_x.pVNEnv->Display(pVisualizationData->pVisualizer, 0.01f, NULL, NULL, 0.0f, &bbox));
                    pVisualizationData->pVisualizer->Run();
                    pVisualizationData->pVisualizer->Clear(pVisualizationData->envActors);
                }

                //

                memcpy(x, xOpt, RVLMOTION_TOUCH_NUM_PARAMS * sizeof(float));
            }
        } // Attempts.
        while (outcome != RVLMOTION_TOUCH_OUTCOME_SUCCESS && iAttempt < maxnAttempts);
        switch (outcome)
        {
        case RVLMOTION_TOUCH_OUTCOME_COLLISION:
            strOutcome = "Collision";
            break;
        case RVLMOTION_TOUCH_OUTCOME_SUCCESS:
            strOutcome = "Success";
            break;
        case RVLMOTION_TOUCH_OUTCOME_MISS:
            strOutcome = "Miss";
            break;
        default:
            strOutcome = "Unknown";
        }
        if (outcome != RVLMOTION_TOUCH_OUTCOME_SUCCESS)
            printf("%s\n", strOutcome.data());
        if (fpLog)
        {
            fprintf(fpLog, "Simulation %d, attempts %d, %s\n", pSimParams->idx, iAttempt, strOutcome.data());
            fclose(fpLog);
        }
        if (bVisualization)
            pVisualizer->Clear();
        iSimulation++;
        iTask++;
        if (iTask == sessionSize)
        {
            ClearSession(touches_E, contacts);
            memset(x, 0, RVLMOTION_TOUCH_NUM_PARAMS * sizeof(float));
            iTask = 0;
        }
    } // Simulations.
    delete[] path.Element;

    // Sample(10000, 0.1f, samples);
    //{
    //     model_gt.pVNEnv->Display(pVisualizer, 0.02f, model_gt.d);
    //     Array<Point> visPts;
    //     visPts.Element = new Point[samples.n];
    //     visPts.n = samples.n;
    //     for (iSample = 0; iSample < samples.n; iSample++)
    //     {
    //         RVLCOPY3VECTOR(samples.Element[iSample].P, visPts.Element[iSample].P);
    //     }
    //     pVisualizer->DisplayPointSet<float, Point>(visPts, darkGreen, 4.0f);
    //     pVisualizer->Run();
    // }

    ///

    pVisualizationData->bOptimization = bVisualizeOptimization;
    pVisualizationData->bContacts = bVisualizeContacts;

    // delete[] plane_E;
    // delete[] touches_W.Element;
    delete[] samples.Element;
}

void Touch::SimulateVisionWithError(
    float *x,
    bool bNewRndx)
{
    // Constants.

    float vertexDistThr = 0.8f * maxReconstructionError;
    float vertexDistThr2 = vertexDistThr * vertexDistThr;

    //

    float *l = x;
    float *pgz = l + 4;
    float *phz = pgz + 1;
    float *phi = phz + 1;
    float *s = phi + 3;
    float *c = s + 3;

    // Simulated error.

    // Array<Point> visPts;
    // visPts.Element = new Point[1000];
    // Point* pVisPt = visPts.Element;
    // for (i = 0; i < 1000; i++, pVisPt++)
    //     PseudoRndSampleUnitSphere(pVisPt->P, rndVal, iRndVal);
    // visPts.n = pVisPt - visPts.Element;
    // pVisualizer->DisplayPointSet<float, Point>(visPts, green, 6.0f);
    // delete[] visPts.Element;
    // pVisualizer->Run();
    // pVisualizer->Clear();

    Pose3D pose_C_E = model_gt.pose_C_E;
    Pose3D pose_C_E_e;
    float a[3];
    float b;
    float a_[3];
    float b_;
    float D[9];
    float invD[9];
    float M[9], V[3];
    float P_E[3];
    int iVertex;
    float maxVertexDist;
    float U[3];
    do
    {
        int i;
        if (bNewRndx)
        {
            for (i = 0; i < 4; i++)
                l[i] = stdl[i] * GaussPseudoRandBM<float>(rndVal, iRndVal);
            *pgz = stdgz * GaussPseudoRandBM<float>(rndVal, iRndVal);
            *phz = stdhz * GaussPseudoRandBM<float>(rndVal, iRndVal);
            for (i = 0; i < 3; i++)
            {
                s[i] = stds * GaussPseudoRandBM<float>(rndVal, iRndVal);
                // c[i] = stdc * GaussPseudoRandBM<float>(rndVal, iRndVal);
                c[i] = 0.0f;
            }
            PseudoRndSampleUnitSphere(U, rndVal, iRndVal);
            int iTmp;
            float ph = stdphirad * GaussPseudoRandBM<float>(rndVal, iRndVal);
            RVLSCALE3VECTOR(U, ph, phi);
        }
        float gz = *pgz;
        float hz = *phz;

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

        float ph = sqrt(RVLDOTPRODUCT3(phi, phi));
        RVLSCALE3VECTOR2(phi, ph, U);
        float Re[9];
        AngleAxisToRot<float>(U, ph, Re);
        float invRe[9];
        RVLCOPYMX3X3T(Re, invRe);
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

        // Auxiliary parameters for vertex and plane transformation.

        AuxParams(&model_e, &model_gt, gz, hz, a, b, a_, b_, D, invD, M, V);

        // Compute vertices estimated by the vision system from the true vertices.

        MOTION::Vertex *pVertex = vertices.Element;
        Vector3<float> *pVertex_e = model_e.pEnvSolidParams->vertex;
        float dP[3];
        float dist2;
        float maxDist2 = 0;
        for (iVertex = 0; iVertex < vertices.n; iVertex++, pVertex++, pVertex_e++)
        {
            VisionVertex(pVertex->P, a, b, invD, pose_C_E, pose_C_E_e, pVertex_e->Element, P_E);
            RVLDIF3VECTORS(pVertex_e->Element, P_E, dP);
            dist2 = RVLDOTPRODUCT3(dP, dP);
            // if (dist2 > vertexDistThr2)
            if (dist2 > maxDist2)
                maxDist2 = dist2;
        }
        maxVertexDist = sqrt(maxDist2);
        int debug = 0;
    } while (bNewRndx && maxVertexDist > vertexDistThr);

    //// a <- gz / (kappae * zne) * Z_E_C.T

    // float *Z_E_C = pose_C_E.R + 6;
    // fTmp = gz / kappazne;
    // RVLSCALE3VECTOR(Z_E_C, fTmp, a);

    //// b <- a * (t_C_E + s) + 1 + hz / (kappae * zne)
    //
    // b = RVLDOTPRODUCT3(a, pose_C_E.t) + 1.0f + hz / kappazne;

    //// a_ <- gz / (kappa * zn) * Z_C_E_e.T

    // float Z_C_E_e[3];
    // RVLCOPYCOLMX3X3(pose_C_E_e.R, 2, Z_C_E_e);
    // fTmp = gz / kappazn;
    // RVLSCALE3VECTOR(Z_C_E_e, fTmp, a_);

    //// b_ <- -a_ * t_E_C_e + kappazne / kappazn

    // b_ = -RVLDOTPRODUCT3(a_, pose_C_E_e.t) + kappazne / kappazn;

    //// D <- R_C_E * K.inv() * Ke * R_C_E_e.T

    ////cv::Mat cvD(3, 3, CV_32FC1);
    ////float *D = (float*)(cvD.data);
    // float M3x3Tmp1[9];
    // RVLMXMUL3X3T2(Ke, pose_C_E_e.R, M3x3Tmp1);
    // float invK[9];
    // InvIntrinsicCameraMatrix(K, invK);
    // float M3x3Tmp2[9];
    // RVLMXMUL3X3(pose_C_E.R, invK, M3x3Tmp2);
    // RVLMXMUL3X3(M3x3Tmp2, M3x3Tmp1, D);

    //// invD <- R_C_E_e * Ke.inv() * K * R_C_E.T

    // RVLMXMUL3X3T2(K, pose_C_E.R, M3x3Tmp1);
    // float invKe[9];
    // InvIntrinsicCameraMatrix(Ke, invKe);
    // RVLMXMUL3X3(pose_C_E_e.R, invKe, M3x3Tmp2);
    // RVLMXMUL3X3(M3x3Tmp2, M3x3Tmp1, invD);

    /// Compute planes estimated by the vision system from the true planes.

    int iSurface;
    float V3Tmp[3];
    float r;
    MOTION::PlanarSurface *pSurface = surfaces.Element;
    MOTION::Plane Plane_E;
    MOTION::Plane *pPlane_e = model_e.pEnvSolidParams->plane;
    for (iSurface = 0; iSurface < surfaces.n; iSurface++, pSurface++, pPlane_e++)
    {
        // Plane_E <- surfaces.Element[iSurface].plane transformed to r.f. E

        RVLPLANETRANSF3(pSurface->plane.N, pSurface->plane.d, pose_W_E.R, pose_W_E.t, Plane_E.N, Plane_E.d);
        VisionPlane(&Plane_E, a_, b_, D, &pose_C_E, &pose_C_E_e, pPlane_e);

        //// r <- Plane_E.d - Plane_E.N.T * pose_C_E.t

        // r = Plane_E.d - RVLDOTPRODUCT3(Plane_E.N, pose_C_E.t);

        //// model_e.plane[iSurface].N <- (D.T * Plane_E.N - r * a_.T).norm

        // RVLMULMX3X3TVECT(D, Plane_E.N, V3Tmp);
        // RVLSCALE3VECTOR(a_, r, pPlane_e->N);
        // RVLDIF3VECTORS(V3Tmp, pPlane_e->N, pPlane_e->N);
        // RVLNORM3(pPlane_e->N, fTmp);

        //// model_e.plane[iSurface].d <- (b_ * r  + (Plane_E.N.T * D * t_C_E_e) / norm(D.T * Plane_E.N - r * a_.T)

        // pPlane_e->d = (r * b_ + RVLDOTPRODUCT3(V3Tmp, pose_C_E_e.t)) / fTmp;

        // printf("%f %f %f %f\n", pPlane_e->N[0], pPlane_e->N[1], pPlane_e->N[2], pPlane_e->d);
    }

    /// Compute door parameters estimated by the vision system from the true door parameters.

    if (bDoor)
    {
        VisionVertex(model_gt.pose_A_E.t, a, b, invD, pose_C_E, pose_C_E_e, model_e.pose_A_E.t, P_E);
        float P_W[3];
        RVLCOPYCOLMX3X3(model_gt.pose_A_E.R, 2, P_W);
        RVLSUM3VECTORS(model_gt.pose_A_E.t, P_W, P_W);
        VisionVertex(P_W, a, b, invD, pose_C_E, pose_C_E_e, model_e.PAxis_E, P_E);
        UpdateDoorOrientation(&model_e);
    }

    ///

    UpdateVerticesAndPlanes(model_e.pEnvSolidParams);

    // Test vertex-plane consistency.

    // TestVertexPlaneConsistency();

    ////

    // Visualization of the envornoment model reconstructed with error.

    RVLCOLORS
    Visualizer *pVisualizer = pVisualizationData->pVisualizer;
    if (bVisualization)
    {
        pVisualizationData->envActors2 = envSolidx.Visualize(pVisualizer, darkGreen);
        pVisualizationData->envActors2.push_back(pVisualizer->DisplayReferenceFrame(&(model_e.pose_A_E), 0.2f));
    }

    // Environment VN model reconstructed with error.

    int iFeature;

    RECOG::VN_::Feature *pFeatureSrc = model_gt.pVNEnv->featureArray.Element;
    RECOG::VN_::Feature *pFeatureTgt = model_e.pVNEnv->featureArray.Element;
    MOTION::Plane Plane_E_e;
    for (iFeature = 0; iFeature < model_e.pVNEnv->featureArray.n; iFeature++, pFeatureSrc++, pFeatureTgt++)
    {
        RVLPLANETRANSF3(pFeatureSrc->N, pFeatureSrc->d, pose_W_E.R, pose_W_E.t, Plane_E.N, Plane_E.d);
        VisionPlane(&Plane_E, a_, b_, D, &pose_C_E, &pose_C_E_e, &Plane_E_e);
        RVLCOPY3VECTOR(Plane_E_e.N, pFeatureTgt->N);
        pFeatureTgt->d = Plane_E_e.d;
    }

    // Visualize environment VN model.

    // Box<float> bbox;
    // SceneBBox(&(envSolidParams.model0), &bbox);
    // model_e.pVNEnv->Display(pVisualizer, 0.01f, NULL, NULL, 0.0f, &bbox);
    // pVisualizer->Run();

    // Test mapping true planes to the planes reconstructed usint a vision system with error.

    // TestPlaneMapping(&model_e);
}

void Touch::Correct(
    MOTION::TouchModel *pModelSrc,
    float *x,
    MOTION::TouchModel *pModelTgt)
{
    float *l = x;
    float gz = x[4];
    float hz = x[5];
    float *phi = x + 6;
    float *s = phi + 3;
    float *c = s + 3;

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
    float *P0,
    float *a_,
    float b_,
    float *D,
    Pose3D *pPose_C_E_0,
    Pose3D *pPose_C_E_x,
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
    MOTION::Plane *pPlaneSrc,
    float *a,
    float b,
    float *M,
    float *V,
    MOTION::Plane *pPlaneTgt)
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

void Touch::AddContact(
    MOTION::Contact contact,
    MOTION::TouchData *pTouch,
    std::vector<MOTION::Contact> &contacts)
{
    if (contact.type == RVLMOTION_TOUCH_CONTACT_TYPE_POINT_PLANE)
    {
        if (!pTouch->pEnvSolidParams->bPlanesForUpdate[contact.iEnvPlane[0]])
        {
            pTouch->pEnvSolidParams->bPlanesForUpdate[contact.iEnvPlane[0]] = true;
            pTouch->pEnvSolidParams->planesForUpdate.Element[pTouch->pEnvSolidParams->planesForUpdate.n++] = contact.iEnvPlane[0];
        }
        int iPlane;
        for (int i = 0; i < contact.nBoundaryPlanes; i++)
        {
            iPlane = contactBoundaryPlanes[contact.iFirstBoundaryPlane + i];
            if (!pTouch->pEnvSolidParams->bPlanesForUpdate[iPlane])
            {
                pTouch->pEnvSolidParams->bPlanesForUpdate[iPlane] = true;
                pTouch->pEnvSolidParams->planesForUpdate.Element[pTouch->pEnvSolidParams->planesForUpdate.n++] = iPlane;
            }
        }
    }
    else if (contact.type == RVLMOTION_TOUCH_CONTACT_TYPE_PLANE_POINT)
    {
        int iVertex = contact.iEnvVertex[0];
        if (!pTouch->pEnvSolidParams->bVerticesForUpdate[iVertex])
        {
            pTouch->pEnvSolidParams->bVerticesForUpdate[iVertex] = true;
            pTouch->pEnvSolidParams->verticesForUpdate.Element[pTouch->pEnvSolidParams->verticesForUpdate.n++] = iVertex;
        }
    }
    else if (contact.type == RVLMOTION_TOUCH_CONTACT_TYPE_EDGE_EDGE)
    {
        int iVertex;
        for (int i = 0; i < 2; i++)
        {
            iVertex = contact.iEnvVertex[i];
            if (!pTouch->pEnvSolidParams->bVerticesForUpdate[iVertex])
            {
                pTouch->pEnvSolidParams->bVerticesForUpdate[iVertex] = true;
                pTouch->pEnvSolidParams->verticesForUpdate.Element[pTouch->pEnvSolidParams->verticesForUpdate.n++] = iVertex;
            }
        }
    }

    contacts.push_back(contact);
}

void Touch::ContactsEE(
    MOTION::TouchData *pTouch,
    std::vector<MOTION::Contact> &contacts,
    bool bTarget)
{
    pTouch->contact.type = RVLMOTION_TOUCH_CONTACT_TYPE_EDGE_EDGE;
    Solid *pSolid;
    SolidEdge *pEnvEdge;
    int iEnvEdge;
    Pair<float, float> err;
    float orthogonalDist, lateralDist;
    float *NSrc, *NTgt;
    for (int iSolid = 0; iSolid < envSolidx.solids.size(); iSolid++)
    {
        if (bTarget)
            if (iSolid != target.iSolid)
                continue;
        pSolid = envSolidx.solids[iSolid];
        pEnvEdge = pSolid->edges.Element;
        for (iEnvEdge = 0; iEnvEdge < pSolid->edges.n; iEnvEdge++, pEnvEdge++)
        {
            // if (bDebug && pTouch->contact.iToolFeature == 2 && iSolid == 0 && iEnvEdge == 5)
            //     int debug = 0;
            if (!pEnvEdge->bConvex)
                continue;
            if (pEnvEdge->iVertex > pSolid->edges.Element[pEnvEdge->iNext].iVertex)
                continue;
            pTouch->contact.iEnvFeature.a = iSolid;
            pTouch->contact.iEnvFeature.b = iEnvEdge;
            pTouch->contact.iEnvVertex[0] = pSolid->vertices.Element[pEnvEdge->iVertex].iSrcVertex;
            pTouch->contact.iEnvVertex[1] = pSolid->vertices.Element[pSolid->edges.Element[pEnvEdge->iNext].iVertex].iSrcVertex;
            pTouch->contact.iEnvPlane[0] = pSolid->faces.Element[pEnvEdge->iFace].iSrcPlane;
            pTouch->contact.iEnvPlane[1] = pSolid->faces.Element[pSolid->edges.Element[pEnvEdge->iTwin].iFace].iSrcPlane;
            err = Error(pTouch, true, false, &orthogonalDist, &lateralDist);
            if (RVLABS(err.b) > maxReconstructionError || RVLABS(orthogonalDist) > maxReconstructionError || lateralDist > maxReconstructionError)
                continue;
            if (IntersectionWithUncert(pTouch))
                // if (Intersection(pTouch->contact.toolPlane[0].N, pTouch->contact.toolPlane[1].N,
                //     pTouch->pEnvSolidParams->modelx.plane[pTouch->contact.iEnvPlane[0]].N,
                //     pTouch->pEnvSolidParams->modelx.plane[pTouch->contact.iEnvPlane[1]].N))
                continue;
            AddContact(pTouch->contact, pTouch, contacts);
        }
    }
}

void Touch::Contacts(
    MOTION::TouchData *pTouch_E,
    std::vector<MOTION::Contact> &contacts,
    bool bVisualization)
{
    RVLCOLORS

    float x0[RVLMOTION_TOUCH_NUM_PARAMS];
    memset(x0, 0, RVLMOTION_TOUCH_NUM_PARAMS * sizeof(float));
    // UpdateEnvironmentModel(&envSolidParams, &model_e, x0, &model_x);
    UpdatEnvSolidParams(pTouch_E->pEnvSolidParams, NULL, &model_e, x0, &model_x);

    pTouch_E->iFirstContact = contacts.size();

    MOTION::Contact contact;
    Solid *pSolid;
    SolidVertex *pSolidVertex;
    SolidEdge *pEdge, *pToolEdge, *pEnvEdge;
    SolidFace *pSolidFace;
    int iSolid, iFace, iToolEdge, iEnvEdge;
    Pair<float, float> err;
    int iVertexFace[4];
    Array<int> vertexFaces;
    vertexFaces.Element = iVertexFace;
    int iVertexEdge[4];
    Array<int> vertexEdges;
    vertexEdges.Element = iVertexEdge;
    SolidFace *contactFacePtr[4];
    int i;
    int iVertex;
    float *P;

    toolMoved.Move(&(tool.solid), &(pTouch_E->pose));

    if (bVisualization)
        pVisualizationData->robotActors.push_back(toolMoved.Visualize(pVisualizationData->pVisualizer, yellow)[0]);

    if (contact.bMiss = pTouch_E->bMiss)
    {
        contact.type = RVLMOTION_TOUCH_CONTACT_TYPE_PLANE_POINT;
        float V[3];
        RVLSCALE3VECTOR(pTouch_E->V, pTouch_E->t, V);

        // Front faces.

        bool *bFront = new bool[toolMoved.faces.n];
        memset(bFront, 0, toolMoved.faces.n * sizeof(bool));
        pSolidFace = toolMoved.faces.Element;
        for (int iFace = 0; iFace < toolMoved.faces.n; iFace++, pSolidFace++)
            if (RVLDOTPRODUCT3(pSolidFace->N, pTouch_E->V) > 0.0f)
                bFront[iFace] = true;

        // Sweeping edges.

        Pair<MOTION::Plane, bool> *sweepingEdgeData = new Pair<MOTION::Plane, bool>[toolMoved.edges.n];
        Array<int> sweepingEdges;
        sweepingEdges.Element = new int[toolMoved.edges.n];
        sweepingEdges.n = 0;
        float *P1, *P2;
        float dP[3];
        Pair<MOTION::Plane, bool> *pSweepingEdgeData = sweepingEdgeData;
        pToolEdge = toolMoved.edges.Element;
        float fTmp;
        int iSweepingEdge;
        for (iToolEdge = 0; iToolEdge < toolMoved.edges.n; iToolEdge++, pToolEdge++, pSweepingEdgeData++)
        {
            if (!(pSweepingEdgeData->b = (bFront[pToolEdge->iFace] && !bFront[toolMoved.edges.Element[pToolEdge->iTwin].iFace])))
                continue;
            P1 = toolMoved.vertices.Element[pToolEdge->iVertex].P;
            P2 = toolMoved.vertices.Element[toolMoved.edges.Element[pToolEdge->iNext].iVertex].P;
            RVLDIF3VECTORS(P2, P1, dP);
            pSweepingEdgeData = sweepingEdgeData + iToolEdge;
            RVLCROSSPRODUCT3(dP, pTouch_E->V, pSweepingEdgeData->a.N);
            RVLNORM3(pSweepingEdgeData->a.N, fTmp);
            pSweepingEdgeData->a.d = RVLDOTPRODUCT3(pSweepingEdgeData->a.N, P1);
            sweepingEdges.Element[sweepingEdges.n++] = iToolEdge;
        }

        // Visualization of tool motion.

        float V3Tmp[3];
        if (bVisualization)
        {
            RVLVISUALIZER_LINES_INIT(visPts, visLines, sweepingEdges.n)
            Point *pVisPt = visPts.Element;
            Pair<int, int> *pVisLine = visLines.Element;
            for (iSweepingEdge = 0; iSweepingEdge < sweepingEdges.n; iSweepingEdge++, pVisLine++)
            {
                iToolEdge = sweepingEdges.Element[iSweepingEdge];
                pEdge = toolMoved.edges.Element + iToolEdge;
                P1 = toolMoved.vertices.Element[pEdge->iVertex].P;
                RVLCOPY3VECTOR(P1, pVisPt->P);
                pVisPt++;
                RVLDIF3VECTORS(P1, V, pVisPt->P);
                pVisPt++;
                pVisLine->a = 2 * iSweepingEdge;
                pVisLine->b = pVisLine->a + 1;
            }
            pVisualizationData->robotActors.push_back(pVisualizationData->pVisualizer->DisplayLines(visPts, visLines, yellow));
            pVisPt = visPts.Element;
            pVisLine = visLines.Element;
            for (iSweepingEdge = 0; iSweepingEdge < sweepingEdges.n; iSweepingEdge++, pVisLine++)
            {
                iToolEdge = sweepingEdges.Element[iSweepingEdge];
                pEdge = toolMoved.edges.Element + iToolEdge;
                P1 = toolMoved.vertices.Element[pEdge->iVertex].P;
                P2 = toolMoved.vertices.Element[toolMoved.edges.Element[pEdge->iNext].iVertex].P;
                RVLSUM3VECTORS(P1, P2, pVisPt->P);
                RVLSCALE3VECTOR(pVisPt->P, 0.5f, pVisPt->P);
                Point *pVisPt_ = pVisPt;
                pVisPt++;
                RVLSCALE3VECTOR(sweepingEdgeData[iToolEdge].a.N, 0.05f, V3Tmp);
                RVLSUM3VECTORS(pVisPt_->P, V3Tmp, pVisPt->P);
                pVisPt++;
                pVisLine->a = 2 * iSweepingEdge;
                pVisLine->b = pVisLine->a + 1;
            }
            pVisualizationData->robotActors.push_back(pVisualizationData->pVisualizer->DisplayLines(visPts, visLines, yellow));
            pVisualizationData->pVisualizer->Run();
            RVLVISUALIZER_LINES_FREE(visPts, visLines)
        }

        /// Contacts for sweeping edges.

        float W[3];
        float e, abse;
        float mine = 0.0f;
        float l;
        int iContactVertex;
        float *N;
        float *PTgt;
        for (iSweepingEdge = 0; iSweepingEdge < sweepingEdges.n; iSweepingEdge++)
        {
            // Tool sweeping edge - environment vertex contacts.

            // if (iSweepingEdge == 4)
            //     int debug = 0;
            iToolEdge = sweepingEdges.Element[iSweepingEdge];
            pToolEdge = toolMoved.edges.Element + iToolEdge;
            N = sweepingEdgeData[iToolEdge].a.N;
            RVLCROSSPRODUCT3(pTouch_E->V, N, W);
            P1 = toolMoved.vertices.Element[pToolEdge->iVertex].P;
            P2 = toolMoved.vertices.Element[toolMoved.edges.Element[pToolEdge->iNext].iVertex].P;
            RVLDIF3VECTORS(P2, P1, dP);
            l = RVLDOTPRODUCT3(W, dP);
            // for (iSolid = 0; iSolid < envSolidx.solids.size(); iSolid++)
            iSolid = target.iSolid;
            {
                pSolid = envSolidx.solids[iSolid];
                iContactVertex = -1;
                pSolidVertex = pSolid->vertices.Element;
                for (iVertex = 0; iVertex < pSolid->vertices.n; iVertex++, pSolidVertex++)
                {
                    P = pTouch_E->pEnvSolidParams->modelx.vertex[pSolidVertex->iSrcVertex].Element;
                    RVLDIF3VECTORS(P, P1, dP);
                    // RVLDIF3VECTORS(pSolidVertex->P, P1, dP);
                    e = RVLDOTPRODUCT3(N, dP);
                    if (iContactVertex < 0 || e < mine)
                    {
                        iContactVertex = iVertex;
                        mine = e;
                    }
                }
                abse = RVLABS(mine);
                if (abse <= maxReconstructionError)
                {
                    pSolidVertex = pSolid->vertices.Element + iContactVertex;
                    P = pTouch_E->pEnvSolidParams->modelx.vertex[pSolidVertex->iSrcVertex].Element;
                    RVLDIF3VECTORS(P, P1, dP);
                    // RVLDIF3VECTORS(pSolidVertex->P, P1, dP);
                    e = RVLDOTPRODUCT3(W, dP);
                    if (e >= -maxReconstructionError && e <= l + maxReconstructionError)
                    {
                        contact.iToolFeature = iToolEdge;
                        contact.toolPlane[0] = sweepingEdgeData[iToolEdge].a;
                        contact.iEnvFeature.a = iSolid;
                        contact.iEnvFeature.b = iContactVertex;
                        contact.iEnvVertex[0] = pSolidVertex->iSrcVertex;
                        AddContact(contact, pTouch_E, contacts);
                    }
                }
            }

            // Tool sweeping vertex - environment edge contacts.

            iVertex = toolMoved.edges.Element[iToolEdge].iVertex;
            int iEdge0, iEdge;
            iEdge = iEdge0 = toolMoved.vertices.Element[iVertex].iEdge;
            do
            {
                if (iEdge != iToolEdge && sweepingEdgeData[iEdge].b)
                    break;
                pEdge = toolMoved.edges.Element + iEdge;
                iEdge = pEdge->iPrev;
                if (iEdge != iToolEdge && sweepingEdgeData[iEdge].b)
                    break;
                pEdge = toolMoved.edges.Element + iEdge;
                iEdge = pEdge->iTwin;
            } while (iEdge != iEdge0);
            contact.iToolFeature = iVertex;
            contact.toolPlane[0] = sweepingEdgeData[iToolEdge].a;
            contact.toolPlane[1] = sweepingEdgeData[iEdge].a;
            P1 = toolMoved.vertices.Element[iVertex].P;
            PTgt = contact.toolVertex[0];
            RVLCOPY3VECTOR(P1, PTgt);
            PTgt = contact.toolVertex[1];
            RVLDIF3VECTORS(P1, V, PTgt);
            pTouch_E->contact = contact;
            ContactsEE(pTouch_E, contacts, true);
        }
        delete[] bFront;
        delete[] sweepingEdgeData;
        delete[] sweepingEdges.Element;

        /// Contacts for back vertex.

        // Back target plane.

        float csVN;
        float maxcsVN = 0.0f;
        int iBackTargetFace = -1;
        pSolid = envSolidx.solids[target.iSolid];
        for (iFace = 0; iFace < pSolid->faces.n; iFace++)
        {
            pSolidFace = pSolid->faces.Element + iFace;
            csVN = RVLDOTPRODUCT3(pTouch_E->V, pSolidFace->N);
            if (iBackTargetFace < 0 || csVN > maxcsVN)
            {
                maxcsVN = csVN;
                iBackTargetFace = iFace;
            }
        }
        pSolidFace = pSolid->faces.Element + iBackTargetFace;
        MOTION::Plane *pPlane = pTouch_E->pEnvSolidParams->modelx.plane + pSolidFace->iSrcPlane;

        // Back vertex.

        float d, mind;
        int iBackVertex = 0;
        P = toolMoved.vertices.Element[0].P;
        mind = RVLDOTPRODUCT3(pTouch_E->V, P);
        for (iVertex = 1; iVertex < toolMoved.vertices.n; iVertex++)
        {
            P = toolMoved.vertices.Element[iVertex].P;
            d = RVLDOTPRODUCT3(pTouch_E->V, P);
            if (d < mind)
            {
                mind = d;
                iBackVertex = iVertex;
            }
        }
        P = toolMoved.vertices.Element[iBackVertex].P;

        // Contact - back vertex in front of the target.

        e = RVLDOTPRODUCT3(pPlane->N, P) - pPlane->d;
        if (e >= -maxReconstructionError)
        {
            contact.type = RVLMOTION_TOUCH_CONTACT_TYPE_POINT_PLANE;
            contact.iToolFeature = iBackVertex;
            PTgt = contact.toolVertex[0];
            RVLDIF3VECTORS(P, V, PTgt);
            contact.iEnvFeature.a = target.iSolid;
            contact.iEnvFeature.b = iBackTargetFace;
            contact.iEnvPlane[0] = pSolidFace->iSrcPlane;
            contact.nBoundaryPlanes = 0;
            AddContact(contact, pTouch_E, contacts);
        }
    }
    else // if (!pTouch_E->bMiss)
    {
        // Tool vertex - environment plane contacts.

        contact.type = RVLMOTION_TOUCH_CONTACT_TYPE_POINT_PLANE;
        float *PSrc, *PTgt;
        float *NSrc, *NTgt;
        PTgt = contact.toolVertex[0];
        int iContactFace;
        int iEdge, iEdge0;
        MOTION::Plane *pPlane;
        Array<Vector3<float>> vertexEdges_;
        Vector3<float> vertexEdgeMem[4];
        vertexEdges_.Element = vertexEdgeMem;
        float *P2;
        float *V;
        float fTmp;
        int iVertex;
        for (iVertex = 0; iVertex < toolMoved.vertices.n; iVertex++)
        {
            toolMoved.GetVertexEdgesAndFaces(iVertex, vertexEdges, vertexFaces);
            for (iContactFace = 0; iContactFace < 3; iContactFace++)
                contactFacePtr[iContactFace] = toolMoved.faces.Element + iVertexFace[iContactFace];
            PSrc = toolMoved.vertices.Element[iVertex].P;
            vertexEdges_.n = vertexEdges.n;
            for (i = 0; i < vertexEdges.n; i++)
            {
                pEdge = toolMoved.edges.Element + vertexEdges.Element[i];
                P2 = toolMoved.vertices.Element[toolMoved.edges.Element[pEdge->iNext].iVertex].P;
                V = vertexEdges_.Element[i].Element;
                RVLDIF3VECTORS(P2, PSrc, V);
                RVLNORM3(V, fTmp);
            }
            for (iSolid = 0; iSolid < envSolidx.solids.size(); iSolid++)
            {
                pSolid = envSolidx.solids[iSolid];
                pSolidFace = pSolid->faces.Element;
                for (iFace = 0; iFace < pSolid->faces.n; iFace++, pSolidFace++)
                {
                    if (pSolidFace->bCovered)
                        continue;
                    pPlane = pTouch_E->pEnvSolidParams->modelx.plane + pSolidFace->iSrcPlane;
                    if (RVLDOTPRODUCT3(pPlane->N, pTouch_E->V) > -csContactAngleThr)
                        // if (RVLDOTPRODUCT3(pSolidFace->N, pTouch_E->V) > -1e-6)
                        continue;
                    contact.iToolFeature = iVertex;
                    RVLCOPY3VECTOR(PSrc, PTgt);
                    contact.iEnvFeature.a = iSolid;
                    contact.iEnvFeature.b = iFace;
                    contact.iEnvPlane[0] = pSolidFace->iSrcPlane;
                    contact.iFirstBoundaryPlane = contactBoundaryPlanes.size();
                    contact.nBoundaryPlanes = 0;
                    iEdge0 = pSolidFace->iEdge;
                    iEdge = iEdge0;
                    do
                    {
                        pEdge = pSolid->edges.Element + iEdge;
                        contactBoundaryPlanes.push_back(pSolid->faces.Element[pSolid->edges.Element[pEdge->iTwin].iFace].iSrcPlane);
                        contact.nBoundaryPlanes++;
                        iEdge = pEdge->iNext;
                    } while (iEdge != iEdge0);
                    pTouch_E->contact = contact;
                    err = Error(pTouch_E, true);
                    if (RVLABS(err.a) > maxReconstructionError || err.b > maxReconstructionError)
                        continue;
                    if (IntersectionWithUncert(pTouch_E, &vertexEdges_))
                        continue;
                    AddContact(contact, pTouch_E, contacts);
                }
            }
        }

        // Edge - edge contacts.

        contact.type = RVLMOTION_TOUCH_CONTACT_TYPE_EDGE_EDGE;
        pToolEdge = toolMoved.edges.Element;
        int iEndEdgeVertex;
        float *PTgt2 = contact.toolVertex[1];
        for (iToolEdge = 0; iToolEdge < toolMoved.edges.n; iToolEdge++, pToolEdge++)
        {
            iEndEdgeVertex = toolMoved.edges.Element[pToolEdge->iNext].iVertex;
            if (pToolEdge->iVertex > iEndEdgeVertex)
                continue;
            contact.iToolFeature = iToolEdge;
            float *P1 = toolMoved.vertices.Element[pToolEdge->iVertex].P;
            float *P2 = toolMoved.vertices.Element[iEndEdgeVertex].P;
            RVLCOPY3VECTOR(P1, PTgt);
            RVLCOPY3VECTOR(P2, PTgt2);
            pSolidFace = toolMoved.faces.Element + pToolEdge->iFace;
            pPlane = contact.toolPlane;
            RVLCOPY3VECTOR(pSolidFace->N, pPlane->N);
            pPlane->d = pSolidFace->d;
            pSolidFace = toolMoved.faces.Element + toolMoved.edges.Element[pToolEdge->iTwin].iFace;
            pPlane++;
            RVLCOPY3VECTOR(pSolidFace->N, pPlane->N);
            pPlane->d = pSolidFace->d;
            pTouch_E->contact = contact;
            ContactsEE(pTouch_E, contacts);
        }

        // Tool plane - environment vertex contacts.

        contact.type = RVLMOTION_TOUCH_CONTACT_TYPE_PLANE_POINT;
        MOTION::Plane plane;
        SolidFace *pSolidFace_;
        SolidVertex *pSolidVertex_;
        int iVertex_;
        for (iSolid = 0; iSolid < envSolidx.solids.size(); iSolid++)
        {
            pSolid = envSolidx.solids[iSolid];
            pSolidVertex = pSolid->vertices.Element;
            for (iVertex = 0; iVertex < pSolid->vertices.n; iVertex++, pSolidVertex++)
            {
                pSolid->GetVertexEdgesAndFaces(iVertex, vertexEdges, vertexFaces);
                PSrc = pTouch_E->pEnvSolidParams->modelx.vertex[pSolidVertex->iSrcVertex].Element;
                vertexEdges_.n = vertexEdges.n;
                for (i = 0; i < vertexEdges.n; i++)
                {
                    pEdge = pSolid->edges.Element + vertexEdges.Element[i];
                    iVertex_ = pSolid->edges.Element[pEdge->iNext].iVertex;
                    pSolidVertex_ = pSolid->vertices.Element + iVertex_;
                    P2 = pTouch_E->pEnvSolidParams->modelx.vertex[pSolidVertex_->iSrcVertex].Element;
                    V = vertexEdges_.Element[i].Element;
                    RVLDIF3VECTORS(P2, PSrc, V);
                    RVLNORM3(V, fTmp);
                }
                pSolidFace = toolMoved.faces.Element;
                for (iFace = 0; iFace < toolMoved.faces.n; iFace++, pSolidFace++)
                {
                    if (RVLDOTPRODUCT3(pSolidFace->N, pTouch_E->V) < 1e-4)
                        continue;
                    contact.iToolFeature = iFace;
                    NSrc = pSolidFace->N;
                    NTgt = contact.toolPlane[0].N;
                    RVLCOPY3VECTOR(NSrc, NTgt);
                    contact.toolPlane[0].d = pSolidFace->d;
                    contact.iFirstBoundaryPlane = contactToolBoundaryPlanes.size();
                    contact.nBoundaryPlanes = 0;
                    iEdge0 = pSolidFace->iEdge;
                    iEdge = iEdge0;
                    do
                    {
                        pEdge = toolMoved.edges.Element + iEdge;
                        pSolidFace_ = toolMoved.faces.Element + toolMoved.edges.Element[pEdge->iTwin].iFace;
                        RVLCOPY3VECTOR(pSolidFace_->N, plane.N);
                        plane.d = pSolidFace_->d;
                        contactToolBoundaryPlanes.push_back(plane);
                        contact.nBoundaryPlanes++;
                        iEdge = pEdge->iNext;
                    } while (iEdge != iEdge0);
                    contact.iEnvFeature.a = iSolid;
                    contact.iEnvFeature.b = iVertex;
                    contact.iEnvVertex[0] = pSolidVertex->iSrcVertex;
                    pTouch_E->contact = contact;
                    err = Error(pTouch_E, true);
                    if (RVLABS(err.a) > maxReconstructionError || err.b > maxReconstructionError)
                        continue;
                    if (IntersectionWithUncert(pTouch_E, &vertexEdges_))
                        continue;
                    AddContact(contact, pTouch_E, contacts);
                }
            }
        }
    }

    pTouch_E->nContacts = contacts.size() - pTouch_E->iFirstContact;
}

bool Touch::Correction(
    float *xInit,
    Array<MOTION::TouchData> touches_E,
    std::vector<MOTION::Contact> &contacts,
    float *xOpt,
    MOTION::Contact *pContactGT,
    float *x_gt,
    bool bUseGTContacts)
{
    // if (touches_E.n == 5)
    //     int debug = 0;

    // Contacts.

    std::vector<Point> visContactPts;
    std::vector<Pair<int, int>> visContactLines;
    bool bVisualizeOptimization = pVisualizationData->bOptimization;
    SetVisualizeOptimization(false);

    int iTouch;
    bool bTrueContact = false;
    bool bTrueContact_;
    MOTION::TouchData *pTouch_E = touches_E.Element;
    for (iTouch = 0; iTouch < touches_E.n; iTouch++, pTouch_E++)
        if (pTouch_E->iFirstContact < 0)
        {
            Contacts(pTouch_E, contacts, pVisualizationData->bContacts);
            if (pContactGT)
            {
                UpdatEnvSolidParams(pTouch_E->pEnvSolidParams, NULL, &model_e, x_gt, &model_x);
                // UpdateEnvironmentModel(pTouch_E->pEnvSolidParams, &model_e, x_gt, &model_x);
                // RVLCOLORS
                // std::vector<vtkSmartPointer<vtkActor>> envSolidGTActors = envSolidx.Visualize(pVisualizationData->pVisualizer, magenta);
                // pVisualizationData->pVisualizer->Run();
                // pVisualizationData->pVisualizer->Clear(envSolidGTActors);
                int iContact;
                MOTION::Contact *pContact;
                Pair<float, float> err;
                for (int i = 0; i < pTouch_E->nContacts; i++)
                {
                    iContact = pTouch_E->iFirstContact + i;
                    pContact = contacts.data() + iContact;
                    bTrueContact_ = false;
                    if (pTouch_E->bMiss)
                    {
                        pTouch_E->contact = *pContact;
                        // bDebug = true;
                        err = Error(pTouch_E, true);
                        // bDebug = false;
                        if (err.b == 0.0f)
                            bTrueContact_ = true;
                    }
                    else
                    {
                        if (pContact->type == pContactGT->type && pContact->iToolFeature == pContactGT->iToolFeature && pContact->iEnvFeature.a == pContactGT->iEnvFeature.a && pContact->iEnvFeature.b == pContactGT->iEnvFeature.b)
                            bTrueContact_ = true;
                    }
                    if (bTrueContact_)
                    {
                        bTrueContact = true;
                        GTContacts[iTouch] = i;
                        printf("True contact is considered.\n");
                        break;
                    }
                }
                UpdatEnvSolidParams(pTouch_E->pEnvSolidParams, NULL, &model_e, xInit, &model_x);
            }
        }
    if (pContactGT)
    {
        // if(pContactGT->type != RVLMOTION_TOUCH_CONTACT_TYPE_NONE)
        if (!bTrueContact)
            printf("True contact is NOT considered.\n");
    }

    MOTION::TouchData *pNewTouch = touches_E.Element + touches_E.n - 1;

    if (pNewTouch->nContacts == 0)
        return false;

    // Only for debugging purpose!!!

    // pNewTouch->contact = *pContactGT;
    // LM(x_gt, touches_E, x)

    // Visualize contacts.

    if (pVisualizationData->bContacts)
    {
        SetVisualizeOptimization(true);
        int i;
        // for (iTouch = 0; iTouch < touches_E.n; iTouch++)
        iTouch = touches_E.n - 1;
        pTouch_E = touches_E.Element + iTouch;
        for (i = 0; i < pTouch_E->nContacts; i++)
        // i = 0;
        {
            pTouch_E->contact = contacts[pTouch_E->iFirstContact + i];
            Error(pTouch_E);
        }
        pVisualizationData->pVisualizer->Run();
        pVisualizationData->pVisualizer->Clear(pVisualizationData->robotActors);
    }
    pVisualizationData->bOptimization = bVisualizeOptimization;

    /// Contact combinations.

    Array2D<int> touchContactCombinations;
    touchContactCombinations.w = touches_E.n;
    int iTouchContactCombination;
    int *touchContacts_;
    MOTION::TouchSolution *pSolution;
    int iSolution;
    int iCombination;
    Array<int> nTouchContacts;
    nTouchContacts.n = touches_E.n;
    nTouchContacts.Element = new int[nTouchContacts.n];
    pTouch_E = touches_E.Element;
    for (iTouch = 0; iTouch < touches_E.n; iTouch++, pTouch_E++)
        nTouchContacts.Element[iTouch] = (pTouch_E->nContacts > 0 ? pTouch_E->nContacts : 1);
    bool bTooManyCombinations = false;
    int nContactCombinations = 1;
    pTouch_E = touches_E.Element;
    for (iTouch = 0; iTouch < touches_E.n; iTouch++, pTouch_E++)
    {
        nContactCombinations *= pTouch_E->nContacts;
        if (nContactCombinations > maxnContactCombinations)
        {
            bTooManyCombinations = true;
            break;
        }
    }
    if (optimizationMethod == RVLMOTION_TOUCH_OPTIMIZATION_METHOD_BRUTEFORCE)
    {
        if (!bTooManyCombinations)
            Combinations(nTouchContacts, touchContactCombinations);
        else
        {
            touchContactCombinations.h = maxnContactCombinations;
            touchContactCombinations.Element = new int[touchContactCombinations.w * touchContactCombinations.h];
            for (iCombination = 0; iCombination < maxnContactCombinations; iCombination++)
            {
                touchContacts_ = touchContactCombinations.Element + iCombination * touchContactCombinations.w;
                pTouch_E = touches_E.Element;
                for (iTouch = 0; iTouch < touches_E.n; iTouch++, pTouch_E++)
                    RVLRND(pTouch_E->nContacts, rndVal.Element, rndVal.n, iRndVal, touchContacts_[iTouch]);
            }
        }
    }
    else if (optimizationMethod == RVLMOTION_TOUCH_OPTIMIZATION_METHOD_RNDSAMPLING)
    {
        touchContactCombinations.Element = new int[nSamples * touches_E.n];

        // GT contacts.

        int nGTSolutions = 0;
        if (bUseGTContacts && pContactGT)
        {
            nGTSolutions = 1;
            memcpy(touchContactCombinations.Element, GTContacts, touches_E.n * sizeof(int));
        }

        // Random combinations.

        int maxnNewSolutions = nSamples - bestSolutions.n - nGTSolutions;
        if (bTooManyCombinations || nContactCombinations > maxnNewSolutions)
        {
            touchContactCombinations.h = maxnNewSolutions + nGTSolutions;
            for (iSolution = 0; iSolution < maxnNewSolutions; iSolution++)
            {
                touchContacts_ = touchContactCombinations.Element + (iSolution + nGTSolutions) * touchContactCombinations.w;
                pTouch_E = touches_E.Element;
                for (iTouch = 0; iTouch < touches_E.n; iTouch++, pTouch_E++)
                    RVLRND(pTouch_E->nContacts, rndVal.Element, rndVal.n, iRndVal, touchContacts_[iTouch]);
            }
        }
        else
        {
            Array2D<int> touchContactCombinations_;
            Combinations(nTouchContacts, touchContactCombinations_);
            memcpy(touchContactCombinations.Element + nGTSolutions * touchContactCombinations.w, touchContactCombinations_.Element,
                   touchContactCombinations_.w * touchContactCombinations_.h * sizeof(int));
            touchContactCombinations.h = touchContactCombinations_.h + nGTSolutions;
            delete[] touchContactCombinations_.Element;
        }

        // Best solutions so far.

        int iContact;
        MOTION::Contact *pContact;
        Pair<float, float> err;
        float e, mine;
        int iClosestContact;
        for (iSolution = 0; iSolution < bestSolutions.n; iSolution++)
        {
            pSolution = bestSolutions.Element + iSolution;
            touchContacts_ = touchContactCombinations.Element + touchContactCombinations.h * touchContactCombinations.w;
            memcpy(touchContacts_, pSolution->contacts, (touchContactCombinations.w - 1) * sizeof(int));
            // RVLRND(pNewTouch->nContacts, rndVal.Element, rndVal.n, iRndVal, touchContacts_[touchContactCombinations.w - 1]);
            // #ifdef NEVER
            UpdatEnvSolidParams(pNewTouch->pEnvSolidParams, NULL, &model_e, pSolution->x, &model_x);
            iClosestContact = -1;
            pContact = contacts.data() + pNewTouch->iFirstContact;
            for (iContact = 0; iContact < pNewTouch->nContacts; iContact++, pContact++)
            {
                pNewTouch->contact = *pContact;
                err = Error(pNewTouch, true);
                e = RVLMAX(RVLABS(err.a), err.b);
                if (iClosestContact < 0 || e < mine)
                {
                    mine = e;
                    iClosestContact = iContact;
                }
            }
            touchContacts_[touchContactCombinations.w - 1] = iClosestContact;
            // #endif
            touchContactCombinations.h++;
        }
    }
    delete[] nTouchContacts.Element;

    // if (!bDebug)
    //{
    //     delete[] touchContactCombinations.Element;
    //     return false;
    // }

    ///

    // Optimization.

    printf("Optimization...\n");
    float x[RVLMOTION_TOUCH_NUM_PARAMS];
    MOTION::TouchLMError E, minE;
    minE.E = -1.0f;
    int *optimalContacts = new int[touches_E.n]; // Only for debugging prupose!!!
    MOTION::TouchSolution *solution = new MOTION::TouchSolution[touchContactCombinations.h];
    int iOptimalSolution;
    for (iTouchContactCombination = 0; iTouchContactCombination < touchContactCombinations.h; iTouchContactCombination++)
    {
        memcpy(x, xInit, RVLMOTION_TOUCH_NUM_PARAMS * sizeof(float));
        // if (touchContactCombinations.w == 5 && iTouchContactCombination == 500)
        //     int debug = 0;
        touchContacts_ = touchContactCombinations.Element + iTouchContactCombination * touchContactCombinations.w;
        pTouch_E = touches_E.Element;
        for (iTouch = 0; iTouch < touches_E.n; iTouch++, pTouch_E++)
            if (pTouch_E->nContacts > 0)
                pTouch_E->contact = contacts[pTouch_E->iFirstContact + touchContacts_[iTouch]];
        // if (touches_E.Element[0].contact.type == RVLMOTION_TOUCH_CONTACT_TYPE_POINT_PLANE &&
        //     touches_E.Element[0].contact.iToolFeature == 6 && touches_E.Element[0].contact.iEnvFeature.a == 3 && touches_E.Element[0].contact.iEnvFeature.b == 4 &&
        //     touches_E.Element[1].contact.type == RVLMOTION_TOUCH_CONTACT_TYPE_POINT_PLANE &&
        //     touches_E.Element[1].contact.iToolFeature == 6 && touches_E.Element[1].contact.iEnvFeature.a == 3 && touches_E.Element[1].contact.iEnvFeature.b == 4)
        //     int debug = 0;
        // if (bDebug)   // For debugging purpose!!!
        //     memcpy(x, x_gt, RVLMOTION_TOUCH_NUM_PARAMS * sizeof(float));
        LM(x, touches_E, x, &E);
        if (bUseGTContacts && iTouchContactCombination == 0)
        {
            printf("GT contacts:\n");
            for (iTouch = 0; iTouch < touches_E.n; iTouch++)
                PrintContact(contacts.data() + touches_E.Element[iTouch].iFirstContact + touchContacts_[iTouch]);
            printf("E=%f*1e-6 ravg=%f gmax=%f Ex=%f*1e-6\n", 1e6 * E.E, E.ravg, E.gmax, 1e6 * E.Ex);
        }
        // Only for debugging purpose!!!
        // if (touches_E.Element[1].contact.type == RVLMOTION_TOUCH_CONTACT_TYPE_POINT_PLANE &&
        //    touches_E.Element[1].contact.iToolFeature == 4 && touches_E.Element[1].contact.iEnvFeature.a == 4 && touches_E.Element[1].contact.iEnvFeature.b == 1 &&
        //    touches_E.Element[2].contact.type == RVLMOTION_TOUCH_CONTACT_TYPE_POINT_PLANE &&
        //    touches_E.Element[2].contact.iToolFeature == 4 && touches_E.Element[2].contact.iEnvFeature.a == 4 && touches_E.Element[2].contact.iEnvFeature.b == 1)
        //{
        //    pTouch_E = touches_E.Element;
        //    for (iTouch = 0; iTouch < touches_E.n; iTouch++, pTouch_E++)
        //        PrintContact(&(pTouch_E->contact));
        //    PrintX(x);
        //    printf("E=%f ravg=%f gmax=%f Ex=%f*1e-6\n", E.E, E.ravg, E.gmax, 1e6 * E.Ex);
        //}
        //
        if (iTouchContactCombination == 0 || E.E < minE.E)
        {
            minE = E;
            memcpy(xOpt, x, RVLMOTION_TOUCH_NUM_PARAMS * sizeof(float));
            pTouch_E = touches_E.Element;
            for (iTouch = 0; iTouch < touches_E.n; iTouch++, pTouch_E++)
                optimalContacts[iTouch] = pTouch_E->iFirstContact + touchContacts_[iTouch];
            iOptimalSolution = iTouchContactCombination;
        }
        pSolution = solution + iTouchContactCombination;
        memcpy(pSolution->x, x, RVLMOTION_TOUCH_NUM_PARAMS * sizeof(float));
        pSolution->cost = E.E;
        pSolution->contacts = touchContacts_;
        if (iTouchContactCombination % 1000 == 0 && iTouchContactCombination > 0)
            printf("%d/%d\n", iTouchContactCombination, touchContactCombinations.h);
    }
    if (optimizationMethod == RVLMOTION_TOUCH_OPTIMIZATION_METHOD_RNDSAMPLING)
    {
        MOTION::TouchSolution *pTopSolution;
        if (touchContactCombinations.h > nBest)
        {
            std::vector<SortIndex<float>> sortedSolutions;
            sortedSolutions.reserve(touchContactCombinations.h);
            SortIndex<float> solutionIdx;
            for (iSolution = 0; iSolution < touchContactCombinations.h; iSolution++)
            {
                solutionIdx.idx = iSolution;
                solutionIdx.cost = solution[iSolution].cost;
                sortedSolutions.push_back(solutionIdx);
            }
            std::sort(sortedSolutions.begin(), sortedSolutions.end(), SortCompare);
            pTopSolution = bestSolutions.Element;
            for (iSolution = 0; iSolution < nBest; iSolution++, pTopSolution++)
            {
                pSolution = solution + sortedSolutions[iSolution].idx;
                memcpy(pTopSolution->x, pSolution->x, RVLMOTION_TOUCH_NUM_PARAMS * sizeof(float));
                pTopSolution->cost = pSolution->cost;
                memcpy(pTopSolution->contacts, pSolution->contacts, touchContactCombinations.w * sizeof(int));
            }
            bestSolutions.n = nBest;
        }
        else
        {
            pTopSolution = bestSolutions.Element;
            pSolution = solution;
            for (iSolution = 0; iSolution < touchContactCombinations.h; iSolution++, pTopSolution++, pSolution++)
            {
                memcpy(pTopSolution->x, pSolution->x, RVLMOTION_TOUCH_NUM_PARAMS * sizeof(float));
                pTopSolution->cost = pSolution->cost;
                memcpy(pTopSolution->contacts, pSolution->contacts, touchContactCombinations.w * sizeof(int));
            }
            bestSolutions.n = touchContactCombinations.h;
        }
    }
    printf("Optimal solution:\n");
    printf("Solution %d\n", iOptimalSolution);
    for (iTouch = 0; iTouch < touches_E.n; iTouch++)
    {
        pTouch_E = touches_E.Element + iTouch;
        printf("touch %d: scene %d ", iTouch, pTouch_E->pEnvSolidParams->idx);
        MOTION::Contact contact;
        if (pTouch_E->nContacts > 0)
        {
            contact = contacts[optimalContacts[iTouch]];
            PrintContact(&contact);
        }
        else
            printf("no contact\n");
    }
    PrintX(xOpt);
    printf("E=%f*1e-6 ravg=%f gmax=%f Ex=%f*1e-6\n", 1e6 * minE.E, minE.ravg, minE.gmax, 1e6 * minE.Ex);
    delete[] optimalContacts;
    delete[] touchContactCombinations.Element;

    // Only for debugging purpose!!!

    // float EOpt = 0.0f;
    // for (i = 0; i < RVLMOTION_TOUCH_NUM_PARAMS; i++)
    //     EOpt += xOpt[i] * xOpt[i] / varx[i];
    // EOpt *= alpha;

    return true;
}

void Touch::ClearSession(
    std::vector<MOTION::TouchData> &touches,
    std::vector<MOTION::Contact> &contacts)
{
    MOTION::TouchData *pTouch = touches.data();
    int i;
    for (int iTouch = 0; iTouch < touches.size(); iTouch++, pTouch++)
    {
        for (i = 0; i < pTouch->pEnvSolidParams->verticesForUpdate.n; i++)
            pTouch->pEnvSolidParams->bVerticesForUpdate[pTouch->pEnvSolidParams->verticesForUpdate.Element[i]] = false;
        pTouch->pEnvSolidParams->verticesForUpdate.n = 0;
        for (i = 0; i < pTouch->pEnvSolidParams->planesForUpdate.n; i++)
            pTouch->pEnvSolidParams->bPlanesForUpdate[pTouch->pEnvSolidParams->planesForUpdate.Element[i]] = false;
        pTouch->pEnvSolidParams->planesForUpdate.n = 0;
    }
    bestSolutions.n = 0;
    contactBoundaryPlanes.clear();
    contactToolBoundaryPlanes.clear();
    if (bVisualization)
        pVisualizationData->pVisualizer->Clear(pVisualizationData->envActors2);
    RVL_DELETE_ARRAY(scenes.Element);
    scenes.Element = NULL;
    scenes.n = 0;
    contacts.clear();
    touches.clear();
}

void Touch::Reconstruct(
    float *P_E,
    MOTION::TouchModel *pModel_x,
    float *P_E_x,
    float *P_C_x,
    float *invKxIn)
{
    float *invKx;
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
    MOTION::TouchModel *pModel)
{
    int iSurface;
    MOTION::PlanarSurface *pSurface = surfaces_.Element;
    MOTION::Plane Plane_E;
    MOTION::Plane *pPlane_e = pModel->pEnvSolidParams->plane;
    for (iSurface = 0; iSurface < surfaces_.n; iSurface++, pSurface++, pPlane_e++)
        RVLPLANETRANSF3(pSurface->plane.N, pSurface->plane.d, pose.R, pose.t, pPlane_e->N, pPlane_e->d);

    int iVertex;
    MOTION::Vertex *pVertex = vertices_.Element;
    Vector3<float> *pVertex_e = pModel->pEnvSolidParams->vertex;
    for (iVertex = 0; iVertex < vertices_.n; iVertex++, pVertex++, pVertex_e++)
        RVLTRANSF3(pVertex->P, pose.R, pose.t, pVertex_e->Element);
}

void Touch::UpdateVerticesAndPlanes(SolidParams *pModel)
{
    // Vertices.

    int iVertex;
    MOTION::Vertex *pVertex = vertices.Element;
    Vector3<float> *pVertexSrc = pModel->vertex;
    SolidVertex *pVertexTgt;
    int i;
    Pair<int, int> *pSolidVertexIdx;
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
    MOTION::PlanarSurface *pSurface = surfaces.Element;
    MOTION::Plane *pPlaneSrc = pModel->plane;
    RECOG::VN_::Feature *pFeature;
    SolidFace *pFace;
    Pair<int, int> *pSolidFaceIdx;
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

void Touch::CopyVerticesAndPlanesFromSolid()
{
    // Vertices.

    int iVertex;
    MOTION::Vertex *pVertexTgt = vertices.Element;
    SolidVertex *pVertexSrc;
    Pair<int, int> *pSolidVertexIdx;
    for (iVertex = 0; iVertex < vertices.n; iVertex++, pVertexTgt++)
    {
        pSolidVertexIdx = pVertexTgt->solidVertices.Element;
        pVertexSrc = envSolid.solids[pSolidVertexIdx->a]->vertices.Element + pSolidVertexIdx->b;
        RVLCOPY3VECTOR(pVertexSrc->P, pVertexTgt->P);
    }

    // Planes.

    int iSurface;
    MOTION::PlanarSurface *pSurface = surfaces.Element;
    SolidFace *pFace;
    Pair<int, int> *pSolidFaceIdx;
    for (iSurface = 0; iSurface < surfaces.n; iSurface++, pSurface++)
    {
        pSolidFaceIdx = pSurface->solidFaces.Element;
        pFace = envSolid.solids[pSolidFaceIdx->a]->faces.Element + pSolidFaceIdx->b;
        RVLCOPY3VECTOR(pFace->N, pSurface->plane.N);
        pSurface->plane.d = pFace->d;
    }
}

void Touch::UpdateDoorAxis(MOTION::TouchModel *pModel0)
{
    // CorrectVertex(pModel0->pose_A_E.t, )
}

void Touch::UpdateDoorOrientation(MOTION::TouchModel *pModel)
{
    float R_E_A_e[9];
    float *X_A_E_e = R_E_A_e;
    float *Y_A_E_e = R_E_A_e + 3;
    float *Z_A_E_e = R_E_A_e + 6;
    float *N = pModel->pEnvSolidParams->plane[doorRefSurfaceIdx].N;
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

void Touch::UpdatEnvSolidParams(
    MOTION::TouchEnvModel *pEnvModelIn,
    Array<MOTION::TouchData> *pTouches,
    MOTION::TouchModel *pModel0,
    float *x,
    MOTION::TouchModel *pModelx)
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

    int nTouches = (pTouches ? pTouches->n : 0);
    if (pEnvModelIn)
        nTouches++;
    MOTION::TouchEnvModel *pEnvModel;
    bool bAll;
    int i;
    int iTouch;
    int iVertex;
    int iSurface;
    Vector3<float> *pVertexSrc;
    Vector3<float> *pVertexTgt;
    MOTION::Plane *pPlaneSrc;
    MOTION::Plane *pPlaneTgt;
    for (iTouch = 0; iTouch < nTouches; iTouch++)
    {
        if (bAll = (iTouch == nTouches - 1 && pEnvModelIn))
            pEnvModel = pEnvModelIn;
        else
            pEnvModel = pTouches->Element[iTouch].pEnvSolidParams;

        // bAll = true;    // Only for debugging purpose!!!

        // Update vertices.

        if (bAll)
        {
            pVertexSrc = pEnvModel->model0.vertex;
            pVertexTgt = pEnvModel->modelx.vertex;
            for (iVertex = 0; iVertex < vertices.n; iVertex++, pVertexSrc++, pVertexTgt++)
                CorrectVertex(pVertexSrc->Element, a_, b_, D, &(pModel0->pose_C_E), &(pModelx->pose_C_E), pVertexTgt->Element);
        }
        else
            for (i = 0; i < pEnvModel->verticesForUpdate.n; i++)
            {
                iVertex = pEnvModel->verticesForUpdate.Element[i];
                pVertexSrc = pEnvModel->model0.vertex + iVertex;
                pVertexTgt = pEnvModel->modelx.vertex + iVertex;
                CorrectVertex(pVertexSrc->Element, a_, b_, D, &(pModel0->pose_C_E), &(pModelx->pose_C_E), pVertexTgt->Element);
            }

        // Update surfaces.

        if (bAll)
        {
            pPlaneSrc = pEnvModel->model0.plane;
            pPlaneTgt = pEnvModel->modelx.plane;
            for (iSurface = 0; iSurface < surfaces.n; iSurface++, pPlaneSrc++, pPlaneTgt++)
                CorrectPlane(pPlaneSrc, a, b, M, V, pPlaneTgt);
        }
        else
            for (i = 0; i < pEnvModel->planesForUpdate.n; i++)
            {
                iSurface = pEnvModel->planesForUpdate.Element[i];
                pPlaneSrc = pEnvModel->model0.plane + iSurface;
                pPlaneTgt = pEnvModel->modelx.plane + iSurface;
                CorrectPlane(pPlaneSrc, a, b, M, V, pPlaneTgt);
            }
    }
}

void Touch::UpdateEnvironmentModel(
    MOTION::TouchEnvModel *pEnvModel,
    MOTION::TouchModel *pModel0,
    float *x,
    MOTION::TouchModel *pModelx)
{
    UpdatEnvSolidParams(pEnvModel, NULL, pModel0, x, pModelx);

#ifdef NEVER
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

    // fTmp = gz / pModel0->kappazn;
    // float Z_C_E_x[3];
    // RVLCOPYCOLMX3X3(pModelx->pose_C_E.R, 2, Z_C_E_x);
    // RVLSCALE3VECTOR(Z_C_E_x, fTmp, a);

    //// b <- a * t_E_C_x + 1 + hz / kappazne

    // b = RVLDOTPRODUCT3(a, pModelx->pose_C_E.t) + 1.0f + hz / pModel0->kappazn;

    //// invD <- pModel0->pose_C_E.R * pModel0->K.inv() * pModelx->K * pModelx->pose_C_E.R.T

    // float M3x3Tmp1[9];
    // RVLMXMUL3X3T2(pModelx->K, pModelx->pose_C_E.R, M3x3Tmp1);
    // float invK0[9];
    // InvIntrinsicCameraMatrix(pModel0->K, invK0);
    // float M3x3Tmp2[9];
    // RVLMXMUL3X3(pModel0->pose_C_E.R, invK0, M3x3Tmp2);
    // RVLMXMUL3X3(M3x3Tmp2, M3x3Tmp1, invD);

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

    // int iFeature;
    // RECOG::VN_::Feature* pFeatureSrc, * pFeatureTgt;
    // MOTION::Plane* plane_E = new MOTION::Plane[pModel0->pVNEnv->featureArray.n];
    // MOTION::Plane* pPlane_E;
    // for (iFeature = 0; iFeature < pModel0->pVNEnv->featureArray.n; iFeature++)
    //{
    //     pFeatureSrc = pModel0->pVNEnv->featureArray.Element + iFeature;
    //     pFeatureTgt = pModelx->pVNEnv->featureArray.Element + iFeature;

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
    Vector3<float> *pVertexSrc = pModel0->vertex;
    Vector3<float> *pVertexTgt = pModelx->vertex;
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
    MOTION::Plane *pPlaneSrc = pModel0->plane;
    MOTION::Plane *pPlaneTgt = pModelx->plane;
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
#endif

    // Update door axis.

    float gz = x[4];
    float hz = x[5];
    float a[3], a_[3];
    float b, b_;
    float D[9], invD[9];
    float M[9], V[3];
    AuxParams(pModel0, pModelx, gz, hz, a, b, a_, b_, D, invD, M, V);
    if (bDoor)
    {
        float V3Tmp[3];
        CorrectVertex(pModel0->pose_A_E.t, a_, b_, D, &(pModel0->pose_C_E), &(pModelx->pose_C_E), pModelx->pose_A_E.t);
        RVLCOPYCOLMX3X3(pModel0->pose_A_E.R, 2, V3Tmp);
        RVLSUM3VECTORS(pModel0->pose_A_E.t, V3Tmp, V3Tmp);
        CorrectVertex(V3Tmp, a_, b_, D, &(pModel0->pose_C_E), &(pModelx->pose_C_E), pModelx->PAxis_E);
    }

    // Update vertices and planes.

    UpdateVerticesAndPlanes(&(pEnvModel->modelx));

    // Test vertex-plane consistency.

    // TestVertexPlaneConsistency();
}

void Touch::UpdateEnvironmentVNModel(
    MOTION::TouchModel *pModel0,
    float *x,
    MOTION::TouchModel *pModelx)
{
    float gz = x[4];
    float hz = x[5];

    float a[3], a_[3];
    float b, b_;
    float D[9], invD[9];
    float M[9], V[3];
    AuxParams(pModel0, pModelx, gz, hz, a, b, a_, b_, D, invD, M, V);

    int iFeature;
    // MOTION::Plane *plane_E = new MOTION::Plane[pModel0->pVNEnv->featureArray.n];
    // MOTION::Plane *pPlane_E;
    float V3Tmp[3];
    float fTmp;
    RECOG::VN_::Feature *pFeatureSrc = pModel0->pVNEnv->featureArray.Element;
    RECOG::VN_::Feature *pFeatureTgt = pModelx->pVNEnv->featureArray.Element;
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

void Touch::RndTouchPoint(MOTION::TouchPoint &touchPt)
{
    // Choose a panel.

    if (touchPt.iPanel < 0)
        RVLRND(4, rndVal.Element, 1000000, iRndVal, touchPt.iPanel);
    Solid *pPanel = envSolidx.solids[touchPt.iPanel];

    // Choose a face.

    SolidFace *pFace;
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
    SolidEdge *pEdge;
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

void Touch::DoorRefFrame(
    Solid *pEnvSolid,
    Pose3D &pose_D_W,
    float *R_W_D)
{
    int iDoorSolid = 4;
    int iDoorRefVertex[] = {7, 4, 1, 6};
    float axisDir[] = {-1.0f, 1.0f, -1.0f};
    float *Pref[4];
    Solid *pDoorSolid = pEnvSolid->solids[iDoorSolid];
    int iRefVertex;
    for (iRefVertex = 0; iRefVertex < 4; iRefVertex++)
        Pref[iRefVertex] = pDoorSolid->vertices.Element[iDoorRefVertex[iRefVertex]].P;
    RVLCOPY3VECTOR(Pref[0], pose_D_W.t);
    float axis[3];
    cv::Mat cvR_D_W(3, 3, CV_32FC1, pose_D_W.R);
    float fTmp;
    for (int iAxis = 0; iAxis < 3; iAxis++)
    {
        iRefVertex = iAxis + 1;
        RVLDIF3VECTORS(Pref[iRefVertex], Pref[0], axis);
        RVLNORM3(axis, fTmp);
        RVLSCALE3VECTOR(axis, axisDir[iAxis], axis);
        RVLCOPYTOCOL3(axis, iAxis, pose_D_W.R);
    }
    cv::Mat cvR_W_D = cvR_D_W.inv();
    float *R_W_D_ = (float *)(cvR_W_D.data);
    RVLCOPYMX3X3(R_W_D_, R_W_D);
}

void Touch::OpenDoorContactPoint(
    Pose3D pose_Ek_W_gt,
    Pose3D pose_E_W,
    Pose3D &pose_Ek_E,
    Pose3D &pose_D_E)
{
    // Tool orientation w.r.t. E.

    RVLMXMUL3X3T1(pose_E_W.R, pose_Ek_W_gt.R, pose_Ek_E.R);

    // Tool reference point.

    int iToolRefVertex[] = {4, 7};
    float *PTool[2];
    for (int iRefVertex = 0; iRefVertex < 2; iRefVertex++)
        PTool[iRefVertex] = tool.solid.vertices.Element[iToolRefVertex[iRefVertex]].P;
    float PToolRef_Ek[3];
    RVLSUM3VECTORS(PTool[0], PTool[1], PToolRef_Ek);
    RVLSCALE3VECTOR(PToolRef_Ek, 0.5f, PToolRef_Ek);
    float PToolRef_W_gt[3];
    RVLTRANSF3(PToolRef_Ek, pose_Ek_W_gt.R, pose_Ek_W_gt.t, PToolRef_W_gt);

    // GT Door reference frame w.r.t. world r.f.

    Pose3D pose_Dgt_W;
    float R_W_Dgt[9];
    DoorRefFrame(&envSolid, pose_Dgt_W, R_W_Dgt);

    // Tool reference point w.r.t. door r.f.

    float PToolRef_D[3];
    float V3Tmp[3];
    RVLDIF3VECTORS(PToolRef_W_gt, pose_Dgt_W.t, V3Tmp);
    RVLMULMX3X3VECT(R_W_Dgt, V3Tmp, PToolRef_D);

    // Door reference frame w.r.t. r.f. E.

    float R_E_D[9];
    DoorRefFrame(&envSolidx, pose_D_E, R_E_D);

    // Tool reference point w.r.t. E.

    float PToolRef_E[3];
    RVLTRANSF3(PToolRef_D, pose_D_E.R, pose_D_E.t, PToolRef_E);

    // Tool position w.r.t. E

    RVLMULMX3X3VECT(pose_Ek_E.R, PToolRef_Ek, pose_Ek_E.t);
    RVLDIF3VECTORS(PToolRef_E, pose_Ek_E.t, pose_Ek_E.t);
}

void Touch::RefPts()
{
    Pair<int, int> iRefVertex[RVLMOTION_TOUCH_NUM_REF_PTS];
    Pair<int, int> *pRefVertexIdx = iRefVertex;
    float bound[6];
    int i, j;
    SolidVertex *pVertex = envSolidx.solids[0]->vertices.Element;
    for (i = 0; i < 3; i++)
    {
        for (j = 0; j < 2; j++, pRefVertexIdx++)
        {
            bound[2 * i + j] = pVertex->P[i];
            pRefVertexIdx->a = pRefVertexIdx->b = 0;
        }
    }
    int iPart, iVertex;
    Solid *pPart;
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
                else if (pVertex->P[i] > bound[2 * i + 1])
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
    MOTION::TouchData *pRefPt = refPts.Element;
    pRefVertexIdx = iRefVertex;
    int iEdge, iEdge0;
    SolidEdge *pEdge;
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
    Pose3D &initPose,
    Array<Vector3<float>> &path,
    float *PTgt)
{
    // Parameters.

    float a = 0.03f;                 // m
    float h = 0.03f;                 // m
    float offset = 0.05f;            // m
    float dist = 0.005f;             // m
    float parallelMoveLen = 0.5f;    // m
    float orthogonalMoveLen = 0.10f; // m

    //

    Solid *pPanel = envSolidx.solids[touchPt.iPanel];
    SolidFace *pFace = pPanel->faces.Element + touchPt.iFace;
    SolidEdge *pEdge = pPanel->edges.Element + touchPt.iEdge;

    float *P0, *P1, *P2;
    SolidEdge *pEdge_;
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
    float *X_Bk_E = R_E_Bk;
    float *Y_Bk_E = R_E_Bk + 3;
    float *Z_Bk_E = R_E_Bk + 6;
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
    Pose3D &finalPose,
    float *V,
    float &t,
    int &iLastSegment,
    MOTION::Contact *pContact)
{
    float *P1 = initPose.t;
    int iSegment;
    float *P2;
    finalPose = initPose;
    float dist;
    uchar contactType;
    int iToolContactFeature;
    int iEnvContactPart;
    int iEnvContactFeature;
    for (iSegment = 0; iSegment < path.n; iSegment++)
    {
        P2 = path.Element[iSegment].Element;
        RVLDIF3VECTORS(P2, P1, V);
        dist = sqrt(RVLDOTPRODUCT3(V, V));
        toolMoved.Move(&(tool.solid), &finalPose);
        t = toolMoved.FreeMove(V, &envSolid_E, contactType, iToolContactFeature, iEnvContactPart, iEnvContactFeature);
        // t = toolMoved.FreeMove(V, &envSolid_E, contactType, iToolContactFeature, iEnvContactPart, iEnvContactFeature, bDebug);
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
    if (contactType == RVLSOLID_CONTACT_TYPE_POINT_PLANE)
    {
        pContact->type = RVLMOTION_TOUCH_CONTACT_TYPE_POINT_PLANE;
        pContact->iToolFeature = iToolContactFeature;
        pContact->iEnvFeature.a = iEnvContactPart;
        pContact->iEnvFeature.b = iEnvContactFeature;
        printf("Contact: tool_vertex %d panel %d face %d\n", iToolContactFeature, iEnvContactPart, iEnvContactFeature);
    }
    else if (contactType == RVLSOLID_CONTACT_TYPE_EDGE_EDGE)
    {
        pContact->type = RVLMOTION_TOUCH_CONTACT_TYPE_EDGE_EDGE;
        SolidEdge *pToolEdge = toolMoved.edges.Element + iToolContactFeature;
        pContact->iToolFeature = (pToolEdge->iVertex < toolMoved.edges.Element[pToolEdge->iNext].iVertex ? iToolContactFeature : pToolEdge->iTwin);
        Solid *pSolid = envSolid_E.solids[iEnvContactPart];
        SolidEdge *pEnvEdge = pSolid->edges.Element + iEnvContactFeature;
        pContact->iEnvFeature.a = iEnvContactPart;
        pContact->iEnvFeature.b = (pEnvEdge->iVertex < pSolid->edges.Element[pEnvEdge->iNext].iVertex ? iEnvContactFeature : pEnvEdge->iTwin);
        printf("Contact: tool_edge %d panel %d edge %d\n", pContact->iToolFeature, iEnvContactPart, pContact->iEnvFeature.b);
    }
    else if (contactType == RVLSOLID_CONTACT_TYPE_PLANE_POINT)
    {
        pContact->type = RVLMOTION_TOUCH_CONTACT_TYPE_PLANE_POINT;
        pContact->iToolFeature = iToolContactFeature;
        pContact->iEnvFeature.a = iEnvContactPart;
        pContact->iEnvFeature.b = iEnvContactFeature;
        printf("Contact: tool_face %d panel %d vertex %d\n", iToolContactFeature, iEnvContactPart, iEnvContactFeature);
    }
    else if (contactType == RVLSOLID_CONTACT_TYPE_NONE)
    {
        pContact->type = RVLMOTION_TOUCH_CONTACT_TYPE_NONE;
        printf("No contact.\n");
    }
    else
        printf("Unexpected contact!\n");
}

void Touch::SceneBBox(
    SolidParams *pSolidParams,
    Box<float> *pBBox)
{
    Vector3<float> *pVertex = pSolidParams->vertex;
    InitBoundingBox<float>(pBBox, pVertex->Element);
    pVertex++;
    for (int iVertex = 1; iVertex < vertices.n; iVertex++, pVertex++)
        UpdateBoundingBox<float>(pBBox, pVertex->Element);
}

void Touch::TestCorrection(
    std::vector<MOTION::DoorExperimentParams> &expData,
    std::vector<MOTION::TouchData> &touches)
{
    if (simulationSeed >= 0)
        iRndVal = simulationSeed;

    RVLCOLORS
    Visualizer *pVisualizer = pVisualizationData->pVisualizer;

    // Camera model.

    model_e.camera = camera;
    IntrinsicCameraMatrix(model_e.camera, model_e.K);
    model_e.kappa = kappa;
    model_e.kappazn = kappa * zn;

    // Create tool for touches.

    toolMoved.Clear();
    toolMoved.Copy(&(tool.solid));
    toolMoved.pVisualizer = pVisualizer;

    // Create environment model for representation in r.f. E.

    envSolid_E.Clear();

    //

    float xOpt[RVLMOTION_TOUCH_NUM_PARAMS];
    std::vector<MOTION::TouchData> sessionTouches;
    Array<MOTION::TouchData> sessionTouches_;
    std::vector<MOTION::Contact> contacts;
    contactBoundaryPlanes.reserve(10000);
    contactToolBoundaryPlanes.reserve(10000);
    MOTION::TouchEnvModel *pEnvSolidParams;
    MOTION::DoorExperimentParams *pExpData = expData.data();
    int iSession = -1;
    MOTION::TouchData touch;
    Pose3D pose_Ek_E;
    Pose3D pose_W_E_gt;
    Pose3D pose_W_A;
    std::vector<vtkSmartPointer<vtkActor>> toolActors;
    float V3Tmp[3];
    for (int iScene = 0; iScene < expData.size(); iScene++, pExpData++)
    {
        // Init session.

        if (pExpData->sessionIdx != iSession)
        {
            iSession = pExpData->sessionIdx;
            ClearSession(sessionTouches, contacts);
            model_e.pose_C_E = pExpData->pose_C_E;
        }

        // Actual scene.

        if (iScene == 0)
            CreateScene(pExpData->sxgt, pExpData->sygt, pExpData->szgt, pExpData->rxgt, pExpData->rygt, pExpData->a, pExpData->b, pExpData->c, pExpData->qDeg_gt);
        else
            CreateSceneSolid(pExpData->sxgt, pExpData->sygt, pExpData->szgt, pExpData->rxgt, pExpData->rygt, pExpData->a, pExpData->b, pExpData->c, pExpData->qDeg_gt, true);
        envSolid_E.Copy(&envSolid);
        Pose3D pose_W_0;
        RVLINVTRANSF3D(doorPose.R, doorPose.t, pose_W_A.R, pose_W_A.t);
        RVLCOMPTRANSF3D(pExpData->pose_A_0_gt.R, pExpData->pose_A_0_gt.t, pose_W_A.R, pose_W_A.t, pose_W_0.R, pose_W_0.t);
        // RVLTRANSF3(envSolid.solids[4]->vertices.Element[6].P, pose_W_0.R, pose_W_0.t, V3Tmp);
        // RVLTRANSF3(envSolid.solids[4]->vertices.Element[5].P, pose_W_0.R, pose_W_0.t, V3Tmp);
        RVLCOMPTRANSF3DWITHINV(pExpData->pose_E_0.R, pExpData->pose_E_0.t, pose_W_0.R, pose_W_0.t, pose_W_E_gt.R, pose_W_E_gt.t, V3Tmp);
        envSolid_E.Move(&envSolid, &pose_W_E_gt);
        envSolid_E.Visualize(pVisualizer, black);

        // Reconstructed scene.

        CreateSceneSolid(pExpData->sx, pExpData->sy, pExpData->sz, pExpData->rx, pExpData->ry, pExpData->a, pExpData->b, pExpData->c, pExpData->qDeg, true);

        // Allocate memory for a new scene.

        if (scenes.Element == NULL)
            scenes.Element = new MOTION::TouchEnvModel[sessionSize];
        pEnvSolidParams = scenes.Element + scenes.n;
        pEnvSolidParams->idx = scenes.n;
        scenes.n++;
        pEnvSolidParams->Create(&envSolid, vertices.n, surfaces.n);

        // Associate model_e and model_x with the new scene.

        model_e.pEnvSolidParams = &(pEnvSolidParams->model0);
        model_x.pEnvSolidParams = &(pEnvSolidParams->modelx);

        // Scene model obtained by vision in r.f. E.

        Pose3D pose_A_C = pExpData->pose_A_C;
        RVLINVTRANSF3D(doorPose.R, doorPose.t, pose_W_A.R, pose_W_A.t);
        Pose3D pose_A_E;
        RVLCOMPTRANSF3D(pExpData->pose_C_E.R, pExpData->pose_C_E.t, pose_A_C.R, pose_A_C.t, pose_A_E.R, pose_A_E.t);
        RVLCOMPTRANSF3D(pose_A_E.R, pose_A_E.t, pose_W_A.R, pose_W_A.t, pose_W_E.R, pose_W_E.t);

        // Tranform the environment model to r.f. E

        SetVerticesAndPlanes(surfaces, vertices, pose_W_E, &model_e);
        RECOG::VN_::Feature *pFeatureSrc = model_gt.pVNEnv->featureArray.Element;
        RECOG::VN_::Feature *pFeatureTgt = model_e.pVNEnv->featureArray.Element;
        for (int iFeature = 0; iFeature < model_gt.pVNEnv->featureArray.n; iFeature++, pFeatureSrc++, pFeatureTgt++)
            RVLPLANETRANSF3(pFeatureSrc->N, pFeatureSrc->d, pose_W_E.R, pose_W_E.t, pFeatureTgt->N, pFeatureTgt->d);

        // x <- 0

        float x[RVLMOTION_TOUCH_NUM_PARAMS];
        memset(x, 0, RVLMOTION_TOUCH_NUM_PARAMS * sizeof(float));

        // Initialize the environment model intended for correction.

        UpdateEnvironmentModel(pEnvSolidParams, &model_e, x, &model_x);
        pVisualizationData->envActors2 = envSolidx.Visualize(pVisualizer, darkGreen);
        pVisualizer->Run();

        // Touch and correct.

        for (int iTouch = 0; iTouch < touches.size(); iTouch++)
            if (touches[iTouch].sessionIdx == pExpData->sessionIdx && touches[iTouch].sceneIdx == pExpData->sceneIdx)
            {
                touch = touches[iTouch];
                // toolMoved.Move(&(tool.solid), &(touch.pose));
                // toolMoved.Visualize(pVisualizer, yellow);
                // pVisualizer->Run();
                touch.pEnvSolidParams = pEnvSolidParams;
                sessionTouches.push_back(touch);
                sessionTouches_.n = sessionTouches.size();
                sessionTouches_.Element = sessionTouches.data();
                if (Correction(x, sessionTouches_, contacts, xOpt))
                {
                    pose_Ek_E = touches[iTouch].pose;
                    toolMoved.Move(&(tool.solid), &pose_Ek_E);
                    toolActors = toolMoved.Visualize(pVisualizer, yellow);
                    pVisualizationData->robotActors.insert(pVisualizationData->robotActors.end(), toolActors.begin(), toolActors.end());
                    // for (int iScene_ = 0; iScene_ < scenes.n; iScene_++)
                    int iScene_ = scenes.n - 1;
                    {
                        UpdateEnvironmentModel(scenes.Element + iScene_, &model_e, xOpt, &model_x);
                        pVisualizationData->envActors = envSolidx.Visualize(pVisualizer, green);
                    }
                    UpdateDoorOrientation(&model_x);
                    Box<float> bbox;
                    SceneBBox(&(pEnvSolidParams->model0), &bbox);
                    UpdateEnvironmentVNModel(&model_e, xOpt, &model_x);
                    // pVisualizationData->envActors.push_back(pVisualizer->DisplayReferenceFrame(&(model_x.pose_A_E), 0.2f));
                    // pVisualizationData->envActors.push_back(model_x.pVNEnv->Display(pVisualizationData->pVisualizer, 0.01f, NULL, NULL, 0.0f, &bbox));
                    pVisualizer->Run();
                    pVisualizer->Clear(pVisualizationData->envActors);
                    pVisualizer->Clear(pVisualizationData->robotActors);
                }
                else
                    printf("No touch.\n");
            }

        //// Visualize touches.

        // for (int iTouch = 0; iTouch < touches.size(); iTouch++)
        //     if (touches[iTouch].sessionIdx == pExpData->sessionIdx && touches[iTouch].sceneIdx == pExpData->sceneIdx)
        //     {
        //         pose_Ek_E = touches[iTouch].pose;
        //         toolMoved.Move(&(tool.solid), &pose_Ek_E);
        //         toolMoved.Visualize(pVisualizer, yellow);
        //     }

        //// Visualize the corrected model.

        // UpdateEnvironmentModel(pEnvSolidParams, &model_e, xOpt, &model_x);
        // UpdateDoorOrientation(&model_x);
        // Box<float> bbox;
        // SceneBBox(&(pEnvSolidParams->model0), &bbox);
        // UpdateEnvironmentVNModel(&model_e, xOpt, &model_x);
        // pVisualizationData->pVisualizer->Clear(pVisualizationData->envActors);
        // pVisualizationData->envActors = envSolidx.Visualize(pVisualizationData->pVisualizer, green);
        ////pVisualizationData->envActors.push_back(pVisualizer->DisplayReferenceFrame(&(model_x.pose_A_E), 0.2f));
        ////pVisualizationData->envActors.push_back(model_x.pVNEnv->Display(pVisualizationData->pVisualizer, 0.01f, NULL, NULL, 0.0f, &bbox));
        // pVisualizer->Run();

        pVisualizer->Clear(pVisualizationData->envActors2);
        pVisualizer->Clear(pVisualizationData->robotActors);
    }
}

void Touch::LoadExperimentFileFormat(
    std::string experimentFileHeader,
    std::vector<std::string> &expDataFormat)
{
    std::stringstream ss(experimentFileHeader);
    std::string item;
    while (std::getline(ss, item, ','))
        expDataFormat.push_back(item);
}

void Touch::LoadArray(
    std::stringstream &ss,
    int n,
    float *array_)
{
    std::string value;
    for (int i = 0; i < n; i++)
        if (std::getline(ss, value, ','))
            array_[i] = std::stof(value);
}

void Touch::LoadExperimentDataFromFile(
    std::string experimentData,
    std::vector<std::string> expDataFormat,
    MOTION::DoorExperimentParams *pSample)
{
    std::stringstream ss(experimentData);
    std::string value;
    int iItem;
    std::string item;
    for (iItem = 0; iItem < expDataFormat.size(); iItem++)
    {
        item = expDataFormat[iItem];
        if (item == "idx")
        {
            if (std::getline(ss, value, ','))
                pSample->idx = std::stoi(value);
        }
        if (item == "session_idx")
        {
            if (std::getline(ss, value, ','))
                pSample->sessionIdx = std::stoi(value);
        }
        if (item == "scene_idx")
        {
            if (std::getline(ss, value, ','))
                pSample->sceneIdx = std::stoi(value);
        }
        if (item == "sx")
        {
            if (std::getline(ss, value, ','))
                pSample->sx = std::stof(value);
        }
        else if (item == "sy")
        {
            if (std::getline(ss, value, ','))
                pSample->sy = std::stof(value);
        }
        else if (item == "sz")
        {
            if (std::getline(ss, value, ','))
                pSample->sz = std::stof(value);
        }
        else if (item == "rx")
        {
            if (std::getline(ss, value, ','))
                pSample->rx = std::stof(value);
        }
        else if (item == "ry")
        {
            if (std::getline(ss, value, ','))
                pSample->ry = std::stof(value);
        }
        else if (item == "state_angle")
        {
            if (std::getline(ss, value, ','))
                pSample->qDeg = std::stof(value);
        }
        else if (item == "R_A_S")
            LoadArray(ss, 9, pSample->pose_A_0_gt.R);
        else if (item == "t_A_S")
            LoadArray(ss, 3, pSample->pose_A_0_gt.t);
        else if (item == "z_rot")
        {
            if (std::getline(ss, value, ','))
            {
                float al = DEG2RAD * std::stof(value);
                float cs = cos(al);
                float sn = sin(al);
                RVLROTZ(cs, sn, pSample->pose_A_0_gt.R);
            }
        }
        else if (item == "R_A_C")
            LoadArray(ss, 9, pSample->pose_A_C.R);
        else if (item == "t_A_C")
            LoadArray(ss, 3, pSample->pose_A_C.t);
        else if (item == "R_C_E")
            LoadArray(ss, 9, pSample->pose_C_E.R);
        else if (item == "t_C_E")
            LoadArray(ss, 3, pSample->pose_C_E.t);
        else if (item == "R_E_0" || item == "R_E_0_contact")
            LoadArray(ss, 9, pSample->pose_E_0.R);
        else if (item == "t_E_0" || item == "t_E_0_contact")
            LoadArray(ss, 3, pSample->pose_E_0.t);
    }
}

void Touch::LoadTouch(
    std::string touchFileData,
    std::vector<std::string> touchFileFormat,
    MOTION::TouchData *pTouch)
{
    pTouch->iFirstContact = -1;
    pTouch->bMiss = false;
    std::stringstream ss(touchFileData);
    std::string value;
    int iItem;
    std::string item;
    for (iItem = 0; iItem < touchFileFormat.size(); iItem++)
    {
        item = touchFileFormat[iItem];
        if (item == "session_idx")
        {
            if (std::getline(ss, value, ','))
                pTouch->sessionIdx = std::stoi(value);
        }
        else if (item == "scene_idx")
        {
            if (std::getline(ss, value, ','))
                pTouch->sceneIdx = std::stoi(value);
        }
        if (item == "type")
        {
            if (std::getline(ss, value, ','))
            {
                int type = std::stoi(value);
                if (type == 1)
                    pTouch->bMiss = true;
            }
        }
        else if (item == "R_Ek_E")
            LoadArray(ss, 9, pTouch->pose.R);
        else if (item == "t_Ek_E")
            LoadArray(ss, 3, pTouch->pose.t);
        else if (item == "V")
            LoadArray(ss, 3, pTouch->V);
        else if (item == "t")
        {
            if (std::getline(ss, value, ','))
                pTouch->t = std::stof(value);
        }
    }
}

void Touch::InitVisualizer(
    Visualizer *pVisualizerIn,
    char *cfgFileName)
{
    MOTION::InitVisualizer(pVisualizerIn, pVisualizationData, pMem0);
    pVisualizationData->paramList.m_pMem = pMem0;
    RVLPARAM_DATA *pParamData;
    pVisualizationData->paramList.Init();
    pParamData = pVisualizationData->paramList.AddParam("Touch.Visualize_optimization", RVLPARAM_TYPE_BOOL, &(pVisualizationData->bOptimization));
    pParamData = pVisualizationData->paramList.AddParam("Touch.Visualize_contacts", RVLPARAM_TYPE_BOOL, &(pVisualizationData->bContacts));
    pVisualizationData->paramList.LoadParams(cfgFileName);
}

void Touch::SetVisualizeOptimization(bool bVisualizeOptimization)
{
    pVisualizationData->bOptimization = bVisualizeOptimization;
}

void Touch::PrintContact(MOTION::Contact *pContact)
{
    if (pContact->type == RVLMOTION_TOUCH_CONTACT_TYPE_POINT_PLANE)
    {
        printf("tool_vertex %d panel %d face %d\n", pContact->iToolFeature, pContact->iEnvFeature.a, pContact->iEnvFeature.b);
    }
    else if (pContact->type == RVLMOTION_TOUCH_CONTACT_TYPE_PLANE_POINT)
    {
        if (pContact->bMiss)
            printf("tool_edge %d panel %d vertex %d\n", pContact->iToolFeature, pContact->iEnvFeature.a, pContact->iEnvFeature.b);
        else
            printf("tool_face %d panel %d vertex %d\n", pContact->iToolFeature, pContact->iEnvFeature.a, pContact->iEnvFeature.b);
    }
    else if (pContact->type == RVLMOTION_TOUCH_CONTACT_TYPE_EDGE_EDGE)
    {
        Solid *pEnvSolid = envSolid.solids[pContact->iEnvFeature.a];
        SolidEdge *pEnvEdge = pEnvSolid->edges.Element + pContact->iEnvFeature.b;
        if (pContact->bMiss)
            printf("tool_vertex %d ", pContact->iToolFeature);
        else
        {
            SolidEdge *pToolEdge = tool.solid.edges.Element + pContact->iToolFeature;
            printf("tool_edge %d-%d ", pToolEdge->iVertex, RVLSOLID_EDGE_END_VERTEX(pToolEdge, tool.solid));
        }
        printf("panel %d edge %d-%d\n", pContact->iEnvFeature.a, pEnvEdge->iVertex, RVLSOLID_EDGE_END_VERTEX_(pEnvEdge, pEnvSolid));
    }
    else
        printf("ERROR: Unknown type of contact!\n");
}

void Touch::PrintX(float *x)
{
    printf("x_nrm: ");
    for (int i = 0; i < 12; i++)
        printf("%f ", x[i] / sqrt(varx[i]));
    printf("\n");
}

bool Touch::Intersection(
    float *N1,
    float *N2,
    float *N3,
    float *N4)
{
    // A <- negated matrix whose columns are N2, N3 and N4

    cv::Mat cvA(3, 3, CV_32FC1);
    float *A = (float *)(cvA.data);
    RVLCOPYTOCOL3(N2, 0, A);
    RVLCOPYTOCOL3(N3, 1, A);
    RVLCOPYTOCOL3(N4, 2, A);
    int i;
    for (i = 0; i < 9; i++)
        A[i] = -A[i];

    // b <- N1

    cv::Mat cvb(3, 1, CV_32FC1);
    float *b = (float *)(cvb.data);
    RVLCOPY3VECTOR(N1, b);

    // x <- solution to Ax = b

    cv::Mat cvx(3, 1, CV_32FC1);
    float *x = (float *)(cvx.data);
    cv::solve(cvA, cvb, cvx);

    // If all elements of x are >= 0, then the intersection is an empty set.

    for (i = 0; i < 3; i++)
        if (x[i] < 0)
            return true;
    return false;
}

bool Touch::IntersectionWithUncert(
    MOTION::TouchData *pTouch,
    Array<Vector3<float>> *pEdgeVectors)
{
    if (pTouch->contact.type == RVLMOTION_TOUCH_CONTACT_TYPE_POINT_PLANE ||
        pTouch->contact.type == RVLMOTION_TOUCH_CONTACT_TYPE_PLANE_POINT)
    {
        Plane *pPlane = (pTouch->contact.type == RVLMOTION_TOUCH_CONTACT_TYPE_POINT_PLANE ? pTouch->pEnvSolidParams->modelx.plane + pTouch->contact.iEnvPlane[0] : pTouch->contact.toolPlane);
        float *V;
        for (int i = 0; i < pEdgeVectors->n; i++)
        {
            V = pEdgeVectors->Element[i].Element;
            if (RVLDOTPRODUCT3(pPlane->N, V) < -snMaxOrientErr)
                return true;
        }
        return false;
    }
    if (pTouch->contact.type == RVLMOTION_TOUCH_CONTACT_TYPE_EDGE_EDGE)
    {
        float *P11 = pTouch->contact.toolVertex[0];
        float *P12 = pTouch->contact.toolVertex[1];
        float *P21 = pTouch->pEnvSolidParams->modelx.vertex[pTouch->contact.iEnvVertex[0]].Element;
        float *P22 = pTouch->pEnvSolidParams->modelx.vertex[pTouch->contact.iEnvVertex[1]].Element;
        Plane *pPlane[2][2];
        pPlane[0][0] = pTouch->contact.toolPlane;
        pPlane[0][1] = pTouch->contact.toolPlane + 1;
        pPlane[1][0] = pTouch->pEnvSolidParams->modelx.plane + pTouch->contact.iEnvPlane[0];
        pPlane[1][1] = pTouch->pEnvSolidParams->modelx.plane + pTouch->contact.iEnvPlane[1];
        float V[2][2][3];
        float fTmp;
        RVLDIF3VECTORS(P12, P11, V[0][0]);
        RVLNORM3(V[0][0], fTmp);
        RVLDIF3VECTORS(P22, P21, V[1][0]);
        RVLNORM3(V[1][0], fTmp);
        float N[3];
        RVLCROSSPRODUCT3(V[0][0], V[1][0], N);
        RVLNEGVECT3(V[0][0], V[0][1]);
        RVLNEGVECT3(V[1][0], V[1][1]);
        fTmp = sqrt(RVLDOTPRODUCT3(N, N));
        if (fTmp > 1e-4)
        {
            RVLSCALE3VECTOR2(N, fTmp, N);
            float U[3];
            int sgn[2][2];
            int i, j;
            for (i = 0; i < 2; i++)
                for (j = 0; j < 2; j++)
                {
                    RVLCROSSPRODUCT3(pPlane[i][j]->N, V[i][j], U);
                    RVLNORM3(U, fTmp);
                    fTmp = RVLDOTPRODUCT3(N, U);
                    sgn[i][j] = (fTmp >= snMaxOrientErr ? 1 : (fTmp > -snMaxOrientErr ? 0 : -1));
                }
            int sgn_[2];
            for (i = 0; i < 2; i++)
            {
                if (sgn[i][0] * sgn[i][1] < 0)
                    return true;
                sgn_[i] = sgn[i][0];
                if (sgn[i][1] != 0)
                    sgn_[i] = sgn[i][1];
            }
            return (sgn_[0] * sgn_[1] > 0);
        }
        else
        {
            // Handling of this case is yet to be implemented!
            return false;
        }
    }
}

bool Touch::Intersection(
    Solid *pSolid1,
    int iVertex,
    Solid *pSolid2,
    int iFace)
{
    SolidVertex *pVertex = pSolid1->vertices.Element + iVertex;
    int iEdge, iEdge0;
    iEdge = iEdge0 = pVertex->iEdge;
    SolidEdge *pEdge;
    SolidVertex *pVertex_;
    Array<Vector3<float>> edgeVectors;
    edgeVectors.Element = new Vector3<float>[pSolid1->maxnVertexEdges];
    edgeVectors.n = 0;
    float *V;
    float fTmp;
    do
    {
        pEdge = toolMoved.edges.Element + iEdge;
        V = edgeVectors.Element[edgeVectors.n++].Element;
        RVLCOPY3VECTOR(pEdge->V, V);
        iEdge = toolMoved.edges.Element[pEdge->iPrev].iTwin;
    } while (iEdge != iEdge0);

    SolidFace *pFace = pSolid2->faces.Element + iFace;

    bool bIntersection = Intersection(edgeVectors, pFace->N);

    delete[] edgeVectors.Element;

    return bIntersection;
}

bool Touch::Intersection(
    Array<Vector3<float>> vertexEdgeVectors,
    float *N)
{
    float *V;
    for (int i = 0; i < vertexEdgeVectors.n; i++)
    {
        V = vertexEdgeVectors.Element[i].Element;
        if (RVLDOTPRODUCT3(N, V) < -contactIntersectionThr)
            return true;
    }

    return false;
}

bool Touch::IntersectionEE(
    Solid *pSolid1,
    int iEdge1,
    Solid *pSolid2,
    int iEdge2)
{
    Solid *pSolid[2];
    pSolid[0] = pSolid1;
    pSolid[1] = pSolid2;
    int iEdge[2];
    iEdge[0] = iEdge1;
    iEdge[1] = iEdge2;
    SolidFace *pFace[2][2];
    int i, j;
    SolidEdge *pEdge;
    for (i = 0; i < 2; i++)
    {
        pEdge = pSolid[i]->edges.Element + iEdge[i];
        for (j = 0; j < 2; j++)
        {
            pFace[i][j] = pSolid[i]->faces.Element + pEdge->iFace;
            pEdge = pSolid[i]->edges.Element + pEdge->iTwin;
        }
    }

    Vector3<float> V[2][3];
    float *V_, *N1, *N2, *N3;
    float e;
    float fTmp;
    for (i = 0; i < 2; i++)
        for (j = 0; j < 2; j++)
        {
            V_ = V[i][j].Element;
            N1 = pFace[0][i]->N;
            N2 = pFace[1][j]->N;
            RVLCROSSPRODUCT3(N1, N2, V_);
            fTmp = RVLDOTPRODUCT3(V_, V_);
            if (fTmp >= 1e-7)
            {
                fTmp = sqrt(fTmp);
                RVLSCALE3VECTOR2(V_, fTmp, V_);
                N3 = pFace[0][1 - i]->N;
                if (RVLDOTPRODUCT3(N3, V_) > 0.0f)
                    RVLNEGVECT3(V_, V_)
            }
            else
            {
                e = RVLDOTPRODUCT3(N1, N2);
                if (1.0f - e < 1e-6)
                    return true;
                else if (e + 1.0f < 1e-6)
                    return false;
            }
        }

    Array<Vector3<float>> edgeVectors;
    edgeVectors.n = 3;
    SolidFace *pFace21, *pFace22;
    SolidEdge *pEdge1 = pSolid1->edges.Element + iEdge1;
    for (i = 0; i < 2; i++)
    {
        edgeVectors.n = 0;
        V_ = V[i][2].Element;
        pFace21 = pFace[1][i];
        e = RVLDOTPRODUCT3(pFace21->N, pEdge1->V);
        if (RVLABS(e) < 1e-6)
        {
        }
        if (e < 0.0f)
            RVLCOPY3VECTOR(pEdge1->V, V_)
        else
            RVLNEGVECT3(pEdge1->V, V_)
        pFace22 = pFace[1][1 - i];
        edgeVectors.Element = V[i];
        if (!Intersection(edgeVectors, pFace22->N))
            return false;
    }

    return true;
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
    for (i = 0; i < 4; i++)
        varx[i] = stdl[i] * stdl[i];
    varx[4] = stdgz * stdgz;
    varx[5] = stdhz * stdhz;
    for (i = 0; i < 3; i++)
    {
        varx[6 + i] = stdphirad * stdphirad;
        varx[9 + i] = stds * stds;
        varx[12 + i] = stdc * stdc;
    }
    csContactAngleThr = cos(contactAngleThr * DEG2RAD);
    snMaxOrientErr = sin(maxOrientErrDeg * DEG2RAD);
}

void Touch::AuxParams(
    MOTION::TouchModel *pModel0,
    MOTION::TouchModel *pModelx,
    float gz,
    float hz,
    float *a,
    float &b,
    float *a_,
    float &b_,
    float *D,
    float *invD,
    float *M,
    float *V)
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
    float *PSrc,
    float *a,
    float b,
    float *invD,
    Pose3D pose_C_E,
    Pose3D pose_C_E_e,
    float *PTgt,
    float *P_E)
{
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
    Pose3D *pPose_C_E_e,
    MOTION::Plane *pPlaneTgt)
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
        RECOG::VN_::Feature *pFeature = model_gt.pVNEnv->featureArray.Element + iFeature;
        RECOG::VN_::Feature *pFeaturex = pModel_x->pVNEnv->featureArray.Element + iFeature;
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
    Solid *pSolid;
    SolidEdge *pEdge;
    SolidFace *pFace;
    SolidVertex *pSolidVertex;
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

void Touch::TransformModelVertices(
    MOTION::TouchModel *pModel,
    Pose3D *pose)
{
    // Transform vertices in the model to the pose.
    Vector3<float> *pVertex = pModel->pEnvSolidParams->vertex;
    float tmp[3];
    for (int iVertex = 0; iVertex < vertices.n; iVertex++, pVertex++)
    {
        RVLTRANSF3(pVertex->Element, pose->R, pose->t, tmp);
        RVLCOPY3VECTOR(tmp, pVertex->Element);
    }
}

void Touch::Update_pose_D_A()
{
    float R_D_E[9];
    float *X_D_E = R_D_E;
    float *Y_D_E = R_D_E + 3;
    float *Z_D_E = R_D_E + 6;
    RVL::Array<RVL::SolidVertex> vertices_ = envSolidx.solids[nPanels - 1]->vertices;
    float V3Tmp[3]; float fTmp;
    RVLDIF3VECTORS(vertices_.Element[7].P, vertices_.Element[4].P, X_D_E);
    RVLNORM3(X_D_E, fTmp);
    RVLDIF3VECTORS(vertices_.Element[1].P, vertices_.Element[7].P, Y_D_E);
    RVLNORM3(Y_D_E, fTmp);
    RVLCROSSPRODUCT3(X_D_E, Y_D_E, Z_D_E);
    RVLCOPYMX3X3T(R_D_E, pose_D_E_x.R);
    RVLCOPY3VECTOR(vertices_.Element[7].P, pose_D_E_x.t);

    Pose3D pose_D_A; 
    Pose3D pose_E_A;
    RVLINVTRANSF3D(model_x.pose_A_E.R, model_x.pose_A_E.t, pose_E_A.R, pose_E_A.t);
    // RVLCOMPTRANSF3D(pose_D_E_x.R, pose_D_E_x.t, pose_E_A.R, pose_E_A.t, pose_D_Arot_x.R, pose_D_Arot_x.t);
    RVLCOMPTRANSF3D(pose_E_A.R, pose_E_A.t, pose_D_E_x.R, pose_D_E_x.t, pose_D_Arot_x.R, pose_D_Arot_x.t);
    // RVLCOMPTRANSF3DWITHINV(pose_Arot_A.R, pose_Arot_A.t, pose_D_A.R, pose_D_A.t, pose_D_Arot_x.R, pose_D_Arot_x.t, V3Tmp);

    // // Get DD point
    // Point visPt_E;
    // RVLCOPY3VECTOR(envSolidx.solids[nPanels - 1]->vertices.Element[7].P, visPt_E.P);
    // // Array<Point> visPts;
    // // visPts.n = 1;
    // // visPts.Element = &visPt_E;
    // RVLTRANSF3(visPt_E.P, pose_E_A.R, pose_E_A.t, t_DD_A_x);
}

void Touch::Update_pose_D_0(Pose3D &pose_E_0)
{
    // Get DD point
    // Pose3D pose_D_E;
    // Point t_D_E;
    // RVLCOPY3VECTOR(envSolidx.solids[nPanels - 1]->vertices.Element[7].P, t_D_E.P);

    RVLCOMPTRANSF3D(pose_E_0.R, pose_E_0.t, pose_D_E_x.R, pose_D_E_x.t, pose_D_0_x.R, pose_D_0_x.t);

    // RVLTRANSF3(t_D_E.P, pose_E_0.R, pose_E_0.t, t_DD_0_x);
    // printf("t_DD_0_x: %f %f %f\n", t_DD_0_x[0], t_DD_0_x[1], t_DD_0_x[2]);
}

void Touch::InitSession(RVL::MOTION::DoorExperimentParams *pExpData, bool useGT)
{
    if (simulationSeed >= 0)
        iRndVal = simulationSeed;

    // Camera model.

    model_e.camera = camera;
    IntrinsicCameraMatrix(model_e.camera, model_e.K);
    model_e.kappa = kappa;
    model_e.kappazn = kappa * zn;

    // Create tool for touches.
    RVLCOLORS
    Visualizer *pVisualizer = pVisualizationData->pVisualizer;

    toolMoved.Clear();
    toolMoved.Copy(&(tool.solid));
    toolMoved.pVisualizer = pVisualizer;

    model_e.pose_C_E = pExpData->pose_C_E;

    memset(&xOpt, 0, RVLMOTION_TOUCH_NUM_PARAMS * sizeof(float));

    // CreateScene(pExpData->sx, pExpData->sy, pExpData->sz, pExpData->rx, pExpData->ry, pExpData->a, pExpData->b, pExpData->c, pExpData->qDeg);

    contactBoundaryPlanes.reserve(10000);
    contactToolBoundaryPlanes.reserve(10000);

    if (useGT)
        CreateScene(pExpData->sxgt, pExpData->sygt, pExpData->szgt, pExpData->rxgt, pExpData->rygt, pExpData->a, pExpData->b, pExpData->c, pExpData->qDeg_gt);
    else
        CreateScene(pExpData->sx, pExpData->sy, pExpData->sz, pExpData->rx, pExpData->ry, pExpData->a, pExpData->b, pExpData->c, pExpData->qDeg);

    if (useGT) // Only for ground truth poses generating.
    {
        Pose3D pose_W_A, pose_W_0;
        // RVLINVTRANSF3D(doorPose.R, doorPose.t, pose_W_A.R, pose_W_A.t);
        // RVLCOMPTRANSF3D(pExpData->pose_A_0_gt.R, pExpData->pose_A_0_gt.t, pose_W_A.R, pose_W_A.t, pose_W_0.R, pose_W_0.t);
        
        Pose3D pose_A_Arot;
        RVLINVTRANSF3D(pose_Arot_A.R, pose_Arot_A.t, pose_A_Arot.R, pose_A_Arot.t);
        RVLINVTRANSF3D(doorPose.R, doorPose.t, pose_W_A.R, pose_W_A.t);
        
        Pose3D pose_A_0_gt;
        RVLCOMPTRANSF3D(pExpData->pose_A_0_gt.R, pExpData->pose_A_0_gt.t, pose_A_Arot.R, pose_A_Arot.t, pose_A_0_gt.R, pose_A_0_gt.t);
        RVLCOMPTRANSF3D(pose_A_0_gt.R, pose_A_0_gt.t, pose_W_A.R, pose_W_A.t, pose_W_0.R, pose_W_0.t);



        // Transform the VN model to base r.f. 0
        int iFeature;
        RECOG::VN_::Feature *pFeatureSrc = model_gt.pVNEnv->featureArray.Element;
        float tmp[3];
        for (iFeature = 0; iFeature < model_gt.pVNEnv->featureArray.n; iFeature++, pFeatureSrc++)
        {
            RVLPLANETRANSF3(pFeatureSrc->N, pFeatureSrc->d, pose_W_0.R, pose_W_0.t, tmp, pFeatureSrc->d);
            RVLCOPY3VECTOR(tmp, pFeatureSrc->N);
        }
    
        if (scenes.Element == NULL)
        {
            scenes.Element = new MOTION::TouchEnvModel[sessionSize];
            scenes.n = 0;
            bestSolutions.n = 0;
        }
        envSolidParams_ = scenes.Element + scenes.n;
        envSolidParams_->idx = scenes.n;
        scenes.n++;
        envSolidParams_->Create(&envSolid, vertices.n, surfaces.n);
        model_gt.pEnvSolidParams = &(envSolidParams_->model0);

        SetVerticesAndPlanes(surfaces, vertices, pose_W_0, &model_gt);
        // TransformModelVertices(&model_gt, &pose_W_0);
        
        // RVL_DELETE_ARRAY(scenes.Element);
    }

    // Visualize gripper mesh.
    std::string toolMeshFileName = "/home/RVLuser/rvl-linux/data/Robotiq3Finger_real/mesh.ply";
    if (pToolMesh)
    delete pToolMesh;
    pToolMesh = new Mesh;
    pToolMesh->LoadPolyDataFromPLY((char *)(toolMeshFileName.data()));
    std::string toolPoseFileName = "/home/RVLuser/ferit_ur5_ws/data/Exp-cabinet_detection-20250508/door_detection/T_G_6.npy";
    loadTransfMatrixFromNPY(toolPoseFileName, pose_G_Ek);

}

void Touch::InitScene(RVL::MOTION::DoorExperimentParams *pExpData)
{
    RVLCOLORS
    Visualizer *pVisualizer = pVisualizationData->pVisualizer;

    if (scenes.Element == NULL)
    {
        scenes.Element = new MOTION::TouchEnvModel[sessionSize];
        scenes.n = 0;
        bestSolutions.n = 0;
    }
    
    // MOTION::TouchEnvModel *pEnvSolidParams;
    envSolidParams_ = scenes.Element + scenes.n;
    envSolidParams_->idx = scenes.n;
    scenes.n++;
    envSolidParams_->Create(&envSolid, vertices.n, surfaces.n);
   
    // Associate model_e and model_x with the new scene.
    
    model_e.pEnvSolidParams = &(envSolidParams_->model0);
    model_x.pEnvSolidParams = &(envSolidParams_->modelx);

    CreateSceneSolid(pExpData->sxgt, pExpData->sygt, pExpData->szgt, pExpData->rxgt, pExpData->rygt, pExpData->a, pExpData->b, pExpData->c, pExpData->qDeg_gt, true);
    // CreateSceneSolid(pExpData->sxgt, pExpData->sygt, pExpData->szgt, pExpData->rxgt, pExpData->rygt, pExpData->a, pExpData->b, pExpData->c, 0.0f, true);
    // CopyVerticesAndPlanesFromSolid();

    Pose3D pose_W_0, pose_W_A, pose_W_E_gt;
    float V3Tmp[3];
    
    float q = pExpData->qDeg_gt * DEG2RAD;
    float cs = cos(q);
    float sn = sin(q);
    RVLROTZ(cs, sn, pose_Arot_A.R);
    RVLNULL3VECTOR(pose_Arot_A.t);
    Pose3D pose_A_Arot;
    RVLINVTRANSF3D(pose_Arot_A.R, pose_Arot_A.t, pose_A_Arot.R, pose_A_Arot.t);
    RVLINVTRANSF3D(doorPose.R, doorPose.t, pose_W_A.R, pose_W_A.t);
    
    Pose3D pose_A_0_gt;
    RVLCOMPTRANSF3D(pExpData->pose_A_0_gt.R, pExpData->pose_A_0_gt.t, pose_A_Arot.R, pose_A_Arot.t, pose_A_0_gt.R, pose_A_0_gt.t);
    RVLCOMPTRANSF3D(pose_A_0_gt.R, pose_A_0_gt.t, pose_W_A.R, pose_W_A.t, pose_W_0.R, pose_W_0.t);
    RVLTRANSF3(envSolid.solids[4]->vertices.Element[6].P, pose_W_0.R, pose_W_0.t, V3Tmp);
    printf("Point: %f %f %f\n", V3Tmp[0], V3Tmp[1], V3Tmp[2]);
    RVLTRANSF3(envSolid.solids[4]->vertices.Element[5].P, pose_W_0.R, pose_W_0.t, V3Tmp);
    printf("Point: %f %f %f\n", V3Tmp[0], V3Tmp[1], V3Tmp[2]);

    // RVLCOMPTRANSF3D(pExpData->pose_A_0_gt.R, pExpData->pose_A_0_gt.t, pose_W_A.R, pose_W_A.t, pose_W_0.R, pose_W_0.t);
    RVLCOMPTRANSF3DWITHINV(pExpData->pose_E_0.R, pExpData->pose_E_0.t, pose_W_0.R, pose_W_0.t, pose_W_E_gt.R, pose_W_E_gt.t, V3Tmp);
    envSolid_E.Clear();
    envSolid_E.Copy(&envSolid);
    envSolid_E.Move(&envSolid, &pose_W_E_gt);
    pVisualizer->Clear(gtActors);
    gtActors = envSolid_E.Visualize(pVisualizer, black);
    
    
    // CreateSceneSolid(pExpData->sx, pExpData->sy, pExpData->sz, pExpData->rx, pExpData->ry, pExpData->a, pExpData->b, pExpData->c, pExpData->qDeg, true);
    CreateScene(pExpData->sx, pExpData->sy, pExpData->sz, pExpData->rx, pExpData->ry, pExpData->a, pExpData->b, pExpData->c, pExpData->qDeg);
    CopyVerticesAndPlanesFromSolid();

    Pose3D pose_A_C = pExpData->pose_A_C;
    Pose3D pose_E_0 = pExpData->pose_E_0;
    RVLINVTRANSF3D(doorPose.R, doorPose.t, pose_W_A.R, pose_W_A.t);
    Pose3D pose_A_E;
    RVLCOMPTRANSF3D(pExpData->pose_C_E.R, pExpData->pose_C_E.t, pose_A_C.R, pose_A_C.t, pose_A_E.R, pose_A_E.t);
    RVLCOMPTRANSF3D(pose_A_E.R, pose_A_E.t, pose_W_A.R, pose_W_A.t, pose_W_E.R, pose_W_E.t);
    envSolid_E.Clear();
    envSolid_E.Copy(&envSolid);
    envSolid_E.Move(&envSolid, &pose_W_E);
    // pVisualizer->Clear(xActors);
    // xActors = envSolid_E.Visualize(pVisualizer, darkGreen);

    // envSolidx.Clear();
    // envSolidx.Copy(&envSolid_E);

    // if (bVisualization)
    //     pVisualizer->Run();

    model_e.pose_C_E = pExpData->pose_C_E;
    model_e.pose_A_E = pose_A_E;

    // Tranform the environment model to r.f. E

    SetVerticesAndPlanes(surfaces, vertices, pose_W_E, &model_e);
    RECOG::VN_::Feature *pFeatureSrc = model_gt.pVNEnv->featureArray.Element;
    RECOG::VN_::Feature *pFeatureTgt = model_e.pVNEnv->featureArray.Element;
    for (int iFeature = 0; iFeature < model_gt.pVNEnv->featureArray.n; iFeature++, pFeatureSrc++, pFeatureTgt++)
        RVLPLANETRANSF3(pFeatureSrc->N, pFeatureSrc->d, pose_W_E.R, pose_W_E.t, pFeatureTgt->N, pFeatureTgt->d);

    model_gt.pVNEnv->CopyDescriptor(model_e.d);
        
    memset(x_, 0, RVLMOTION_TOUCH_NUM_PARAMS * sizeof(float));

    UpdateEnvironmentModel(envSolidParams_, &model_e, xOpt, &model_x);
    UpdateDoorOrientation(&model_x);


    // float gz = x_[4];
    // float hz = x_[5];
    // float a[3], a_[3];
    // float b, b_;
    // float D[9], invD[9];
    // float M[9], V[3];
    // AuxParams(&model_gt, &model_e, gz, hz, a, b, a_, b_, D, invD, M, V);
    // RECOG::VN_::Feature *pFeatureSrc = model_gt.pVNEnv->featureArray.Element;
    // RECOG::VN_::Feature *pFeatureTgt = model_e.pVNEnv->featureArray.Element;
    // MOTION::Plane Plane_E;
    // MOTION::Plane Plane_E_e;
    // for (int iFeature = 0; iFeature < model_e.pVNEnv->featureArray.n; iFeature++, pFeatureSrc++, pFeatureTgt++)
    // {
    //     RVLPLANETRANSF3(pFeatureSrc->N, pFeatureSrc->d, pose_W_E.R, pose_W_E.t, Plane_E.N, Plane_E.d);
    //     VisionPlane(&Plane_E, a_, b_, D, &model_e.pose_C_E, &model_e.pose_C_E, &Plane_E_e);
    //     RVLCOPY3VECTOR(Plane_E_e.N, pFeatureTgt->N);
    //     pFeatureTgt->d = Plane_E_e.d;
    // }

    UpdateEnvironmentVNModel(&model_e, xOpt, &model_x);
    // CopyVerticesAndPlanesFromSolid();

    SceneBBox(&(envSolidParams_->modelx), &bbox_);

    pVisualizer->Clear(xActors);
    xActors = envSolidx.Visualize(pVisualizer, darkGreen);

    Update_pose_D_A();
    Update_pose_D_0(pExpData->pose_E_0);

    pVisualizer->renderer->RemoveActor(actor_D_E);
    actor_D_E = pVisualizer->DisplayReferenceFrame(&pose_D_E_x, 0.2f);
    
    if (bVisualization)
        pVisualizer->Run();
    
    // Visualize environment model
    // envSolid_E.Clear();
    // envSolid_E.Copy(&envSolid);
    // envSolid_E.Move(&envSolid, &pose_W_E);
    // envSolid_E.Visualize(pVisualizer, darkGreen);
    // for (int iActor = 0; iActor < pVisualizationData->envActors2.size(); iActor++)
    // {
    //     pVisualizer->renderer->RemoveViewProp(pVisualizationData->envActors2[iActor]);
    // }
    // pVisualizationData->envActors2.clear();
    // pVisualizationData->envActors2 = envSolidx.Visualize(pVisualizer, darkGreen);


    // pVisualizer->Clear(xActors);
    // xActors = envSolidx.Visualize(pVisualizer, darkGreen);
}

void Touch::TestCorrection3(
    MOTION::DoorExperimentParams *pExpData,
    std::vector<MOTION::Contact> &contacts,
    std::vector<MOTION::TouchData> &touches)
{
    RVLCOLORS
    Visualizer *pVisualizer = pVisualizationData->pVisualizer;

    MOTION::TouchEnvModel *envSolidParams;
    envSolidParams = scenes.Element + scenes.n - 1;
    envSolidParams->idx = scenes.n - 1;

    UpdateEnvironmentModel(envSolidParams, &model_e, x_, &model_x);

    // Touch.
    int numTouches = touches.size();
    MOTION::TouchData *pTouch = touches.data() + numTouches - 1;
    Pose3D pose_Ek_E = pTouch->pose;
    toolMoved.Clear();
    toolMoved.Copy(&(tool.solid));
    toolMoved.Move(&(tool.solid), &pose_Ek_E);

    // tool.solid.Visualize(pVisualizer, yellow);
    // pVisualizer->Run();
    vtkNew<vtkActor> actor;

    if (bVisualization)
    {
        std::vector<vtkSmartPointer<vtkActor>> toolActors;

        Pose3D pose_G_E;
        RVLCOMPTRANSF3D(pose_Ek_E.R, pose_Ek_E.t, pose_G_Ek.R, pose_G_Ek.t, pose_G_E.R, pose_G_E.t);

        vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
        double T[16];
        RVLHTRANSFMX(pose_G_E.R, pose_G_E.t, T);
        transform->SetMatrix(T);
        vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
        transformFilter->SetInputData(pToolMesh->pPolygonData);
        transformFilter->SetTransform(transform);
        transformFilter->Update();
        pVisualizer->map = vtkSmartPointer<vtkPolyDataMapper>::New();
        pVisualizer->map->SetInputConnection(transformFilter->GetOutputPort());
        pVisualizer->map->InterpolateScalarsBeforeMappingOff();
        actor->SetMapper(pVisualizer->map);
        pVisualizer->renderer->AddViewProp(actor.GetPointer());

        // Visualize tool
        toolActors = toolMoved.Visualize(pVisualizer, yellow);
        pVisualizationData->robotActors.insert(pVisualizationData->robotActors.end(), toolActors.begin(), toolActors.end());


        // pVisualizationData->envActors.push_back(pVisualizer->DisplayReferenceFrame(&(model_e.pose_A_E), 0.2f));
        // {
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
        //    pVisualizer->Run();
        // }

        pVisualizer->Run();

        // pVisualizer->renderer->RemoveActor(pVisualizer->actor);

        // pVisualizer->Clear();
    }

    Array<MOTION::TouchData> touches_;
    touches_.n = touches.size();
    touches_.Element = touches.data();
    // std::vector<MOTION::Contact> contacts;
    if (Correction(x_, touches_, contacts, xOpt))
    {
        // Visualize the corrected model.

        UpdateEnvironmentModel(envSolidParams, &model_e, xOpt, &model_x);
        UpdateDoorOrientation(&model_x);
        UpdateEnvironmentVNModel(&model_e, xOpt, &model_x);
        Update_pose_D_A();
        Update_pose_D_0(pExpData->pose_E_0);

        // memcpy(x_, xOpt, RVLMOTION_TOUCH_NUM_PARAMS * sizeof(float));

        xActors2 = envSolidx.Visualize(pVisualizationData->pVisualizer, green);
        if (bVisualization)
        {
            // pVisualizationData->envActors.push_back(pVisualizer->DisplayPointSet<float, Point>(visPts, red, 6.0f));
            // pVisualizationData->envActors.push_back(pVisualizer->DisplayReferenceFrame(&(model_x.pose_A_E), 0.2f));
            // pVisualizationData->envActors.push_back(model_x.pVNEnv->Display(pVisualizationData->pVisualizer, 0.01f, NULL, NULL, 0.0f, &bbox));
            pVisualizationData->pVisualizer->Run();
        }
        pVisualizer->Clear(xActors2);
        pVisualizer->Clear(xActors);
        xActors = envSolidx.Visualize(pVisualizer, darkGreen);

        pVisualizer->renderer->RemoveActor(actor_D_E);
        actor_D_E = pVisualizer->DisplayReferenceFrame(&pose_D_E_x, 0.2f);

        // pVisualizer->Clear(pVisualizationData->envActors2);
        pVisualizer->Clear(pVisualizationData->robotActors);
        pVisualizer->renderer->RemoveViewProp(actor.GetPointer());

        // memcpy(x_, xOpt, RVLMOTION_TOUCH_NUM_PARAMS * sizeof(float));
        printf("Correction successful.\n");
    }
    else
        printf("No touch.\n");
}


void Touch::loadTransfMatrixFromNPY(std::string fileName, Pose3D &pose)
{
    cnpy::NpyArray npyData = cnpy::npy_load(fileName);
    double *data = npyData.data<double>();
    double *pData = data;
    double *srcRow;
    float *tgtRow;
    srcRow = pData;
    tgtRow = pose.R;
    int i;
    for (i = 0; i < 3; i++, srcRow += 4, tgtRow += 3)
    {
        RVLCOPY3VECTOR(srcRow, tgtRow);
        pose.t[i] = srcRow[3];
    }
}