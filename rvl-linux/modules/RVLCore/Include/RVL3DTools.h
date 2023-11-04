// V = [x y z]'
#define RVLSET3VECTOR(V, x, y, z) \
    {                             \
        V[0] = x;                 \
        V[1] = y;                 \
        V[2] = z;                 \
    }
// Tgt = Src(3x1)
#define RVLCOPY3VECTOR(Src, Tgt) \
    Tgt[0] = Src[0];             \
    Tgt[1] = Src[1];             \
    Tgt[2] = Src[2];
// Tgt = Src(3x3)
#define RVLCOPYMX3X3(Src, Tgt) \
    Tgt[0] = Src[0];           \
    Tgt[1] = Src[1];           \
    Tgt[2] = Src[2];           \
    Tgt[3] = Src[3];           \
    Tgt[4] = Src[4];           \
    Tgt[5] = Src[5];           \
    Tgt[6] = Src[6];           \
    Tgt[7] = Src[7];           \
    Tgt[8] = Src[8];
// Tgt = Src(3x3)'
#define RVLCOPYMX3X3T(Src, Tgt) \
    Tgt[0] = Src[0];            \
    Tgt[1] = Src[3];            \
    Tgt[2] = Src[6];            \
    Tgt[3] = Src[1];            \
    Tgt[4] = Src[4];            \
    Tgt[5] = Src[7];            \
    Tgt[6] = Src[2];            \
    Tgt[7] = Src[5];            \
    Tgt[8] = Src[8];
// Tgt = -Src(3x1)
#define RVLNEGVECT3(Src, Tgt) \
    Tgt[0] = -Src[0];         \
    Tgt[1] = -Src[1];         \
    Tgt[2] = -Src[2];
// y = i-th column of X(3x3)
#define RVLCOPYCOLMX3X3(X, i, y) \
    y[0] = X[i];                 \
    y[1] = X[3 + i];             \
    y[2] = X[6 + i];
// i-th column of Y(3x3) = x
#define RVLCOPYTOCOL3(x, i, Y) \
    Y[i] = x[0];               \
    Y[3 + i] = x[1];           \
    Y[6 + i] = x[2];
// Tgt(i:i+2, j:j+2) = Src, (Rows of Tgt have n elements)
#define RVLCOPY3BLOCKTOMX(Src, Tgt, i, j, n) \
    {                                        \
        Tgt[n * i + j] = Src[0];             \
        Tgt[n * i + j + 1] = Src[1];         \
        Tgt[n * i + j + 2] = Src[2];         \
        Tgt[n * (i + 1) + j] = Src[3];       \
        Tgt[n * (i + 1) + j + 1] = Src[4];   \
        Tgt[n * (i + 1) + j + 2] = Src[5];   \
        Tgt[n * (i + 2) + j] = Src[6];       \
        Tgt[n * (i + 2) + j + 1] = Src[7];   \
        Tgt[n * (i + 2) + j + 2] = Src[8];   \
}
// Tgt(i:i+2, j:j+2) = Src', (Rows of Tgt have n elements)
#define RVLCOPY3BLOCKTTOMX(Src, Tgt, i, j, n) \
    {                                         \
        Tgt[n * i + j] = Src[0];              \
        Tgt[n * i + j + 1] = Src[3];          \
        Tgt[n * i + j + 2] = Src[6];          \
        Tgt[n * (i + 1) + j] = Src[1];        \
        Tgt[n * (i + 1) + j + 1] = Src[4];    \
        Tgt[n * (i + 1) + j + 2] = Src[7];    \
        Tgt[n * (i + 2) + j] = Src[2];        \
        Tgt[n * (i + 2) + j + 1] = Src[5];    \
        Tgt[n * (i + 2) + j + 2] = Src[8];    \
}
// Z = X(3x3) + Y(3x3)
#define RVLSUMMX3X3(X, Y, Z) \
    Z[0] = X[0] + Y[0];      \
    Z[1] = X[1] + Y[1];      \
    Z[2] = X[2] + Y[2];      \
    Z[3] = X[3] + Y[3];      \
    Z[4] = X[4] + Y[4];      \
    Z[5] = X[5] + Y[5];      \
    Z[6] = X[6] + Y[6];      \
    Z[7] = X[7] + Y[7];      \
    Z[8] = X[8] + Y[8];
// Z = X(3x3) - Y(3x3)
#define RVLDIFMX3X3(X, Y, Z) \
    Z[0] = X[0] - Y[0];      \
    Z[1] = X[1] - Y[1];      \
    Z[2] = X[2] - Y[2];      \
    Z[3] = X[3] - Y[3];      \
    Z[4] = X[4] - Y[4];      \
    Z[5] = X[5] - Y[5];      \
    Z[6] = X[6] - Y[6];      \
    Z[7] = X[7] - Y[7];      \
    Z[8] = X[8] - Y[8];
// Z = X(3x3) + Y(3x3)' (only diagonal + upper triangle are computed)
#define RVLSUMMX3X3T2UT(X, Y, Z) \
    Z[0] = X[0] + Y[0];          \
    Z[1] = X[1] + Y[3];          \
    Z[2] = X[2] + Y[6];          \
    Z[4] = X[4] + Y[4];          \
    Z[5] = X[5] + Y[7];          \
    Z[8] = X[8] + Y[8];
// Z = X(3x3) + Y(3x3) (only diagonal + upper triangle are computed)
#define RVLSUMMX3X3UT(X, Y, Z) \
    Z[0] = X[0] + Y[0];        \
    Z[1] = X[1] + Y[1];        \
    Z[2] = X[2] + Y[2];        \
    Z[4] = X[4] + Y[4];        \
    Z[5] = X[5] + Y[5];        \
    Z[8] = X[8] + Y[8];
// X = 0(3x1)
#define RVLNULL3VECTOR(X) X[0] = X[1] = X[2] = 0.0;
// X = 0(3x3)
#define RVLNULLMX3X3(X) X[0] = X[1] = X[2] = X[3] = X[4] = X[5] = X[6] = X[7] = X[8] = 0.0;
// X = I(3x3)
#define RVLUNITMX3(X)         \
    X[0] = X[4] = X[8] = 1.0; \
    X[1] = X[2] = X[3] = X[5] = X[6] = X[7] = 0.0;
// X = diag(x)
#define RVL3VECTORTODIAGMX(x, X) \
    X[0] = x[0];                 \
    X[4] = x[1];                 \
    X[8] = x[2];                 \
    X[1] = X[2] = X[3] = X[5] = X[6] = X[7] = 0.0;
// X = diag([d1 d2 d3]')
#define RVLDIAGMX3(d1, d2, d3, X) \
    X[0] = d1;                    \
    X[4] = d2;                    \
    X[8] = d3;                    \
    X[1] = X[2] = X[3] = X[5] = X[6] = X[7] = 0.0;
// element in i-th row and j-th column of matrix Mx with nCol columns
#define RVLMXEL(Mx, nCols, i, j) Mx[nCols * i + j]
// Tgt = Src1(3x1) + Src2(3x1)
#define RVLSUM3VECTORS(Src1, Src2, Tgt) \
    Tgt[0] = Src1[0] + Src2[0];         \
    Tgt[1] = Src1[1] + Src2[1];         \
    Tgt[2] = Src1[2] + Src2[2];
// Tgt = Src1(3x1) - Src2(3x1)
#define RVLDIF3VECTORS(Src1, Src2, Tgt) \
    Tgt[0] = Src1[0] - Src2[0];         \
    Tgt[1] = Src1[1] - Src2[1];         \
    Tgt[2] = Src1[2] - Src2[2];
// Tgt = a * Src(3x1)
#define RVLSCALE3VECTOR(Src, a, Tgt) \
    Tgt[0] = a * Src[0];             \
    Tgt[1] = a * Src[1];             \
    Tgt[2] = a * Src[2];
// Tgt = Src(3x1) / a
#define RVLSCALE3VECTOR2(Src, a, Tgt) \
    Tgt[0] = Src[0] / a;              \
    Tgt[1] = Src[1] / a;              \
    Tgt[2] = Src[2] / a;
// Tgt = diag(a) * Src
#define RVLSCALE3VECTOR3(Src, a, Tgt) \
    {                                 \
        Tgt[0] = Src[0] * a[0];       \
        Tgt[1] = Src[1] * a[1];       \
        Tgt[2] = Src[2] * a[2];       \
    }
// Tgt = Src(3x3) * a
#define RVLSCALEMX3X3(Src, a, Tgt) \
    {                              \
        Tgt[0] = a * Src[0];       \
        Tgt[1] = a * Src[1];       \
        Tgt[2] = a * Src[2];       \
        Tgt[3] = a * Src[3];       \
        Tgt[4] = a * Src[4];       \
        Tgt[5] = a * Src[5];       \
        Tgt[6] = a * Src[6];       \
        Tgt[7] = a * Src[7];       \
        Tgt[8] = a * Src[8];       \
}
// Tgt = Src(3x3) / a
#define RVLSCALEMX3X32(Src, a, Tgt) \
    {                               \
        Tgt[0] = Src[0] / a;        \
        Tgt[1] = Src[1] / a;        \
        Tgt[2] = Src[2] / a;        \
        Tgt[3] = Src[3] / a;        \
        Tgt[4] = Src[4] / a;        \
        Tgt[5] = Src[5] / a;        \
        Tgt[6] = Src[6] / a;        \
        Tgt[7] = Src[7] / a;        \
        Tgt[8] = Src[8] / a;        \
}
// Tgt = Src(3x3) * a (only diagonal + upper triangle are computed)
#define RVLSCALEMX3X3UT(Src, a, Tgt) \
    {                                \
        Tgt[0] = a * Src[0];         \
        Tgt[1] = a * Src[1];         \
        Tgt[2] = a * Src[2];         \
        Tgt[4] = a * Src[4];         \
        Tgt[5] = a * Src[5];         \
        Tgt[8] = a * Src[8];         \
}
// TgtCol = a * SrcCol, where SrcCol and TgtCol are the i-th column of 3x3 matrices Src and Tgt respectively
#define RVLSCALECOL3(Src, i, a, Tgt) \
    Tgt[i] = a * Src[i];             \
    Tgt[i + 3] = a * Src[i + 3];     \
    Tgt[i + 6] = a * Src[i + 6];
// dot product of i-th row of A(3x3) and j-th column of B(3x3)
#define RVLMULROWCOL3(A, B, i, j) (A[3 * i + 0] * B[3 * 0 + j] + A[3 * i + 1] * B[3 * 1 + j] + A[3 * i + 2] * B[3 * 2 + j])
// dot product of i-th row of A(3x3) and j-th row of B(3x3)
#define RVLMULROWROW3(A, B, i, j) (A[3 * i + 0] * B[3 * j + 0] + A[3 * i + 1] * B[3 * j + 1] + A[3 * i + 2] * B[3 * j + 2])
// dot product of i-th column of A(3x3) and j-th column of B(3x3)
#define RVLMULCOLCOL3(A, B, i, j) (A[3 * 0 + i] * B[3 * 0 + j] + A[3 * 1 + i] * B[3 * 1 + j] + A[3 * 2 + i] * B[3 * 2 + j])
// y = A(3x3) * x(3x1)
#define RVLMULMX3X3VECT(A, x, y)                    \
    y[0] = A[0] * x[0] + A[1] * x[1] + A[2] * x[2]; \
    y[1] = A[3] * x[0] + A[4] * x[1] + A[5] * x[2]; \
    y[2] = A[6] * x[0] + A[7] * x[1] + A[8] * x[2];
// y = A(3x3)' * x(3x1)
#define RVLMULMX3X3TVECT(A, x, y)                   \
    y[0] = A[0] * x[0] + A[3] * x[1] + A[6] * x[2]; \
    y[1] = A[1] * x[0] + A[4] * x[1] + A[7] * x[2]; \
    y[2] = A[2] * x[0] + A[5] * x[1] + A[8] * x[2];
// invt = -R(3x3)' * t(3x1)
#define RVLINVTRANSL(R, t, invt)                        \
    invt[0] = -R[0] * t[0] - R[3] * t[1] - R[6] * t[2]; \
    invt[1] = -R[1] * t[0] - R[4] * t[1] - R[7] * t[2]; \
    invt[2] = -R[2] * t[0] - R[5] * t[1] - R[8] * t[2];
// y = A(3x3) * x(3x1), where A is a simetric matrix with only diagonal + upper triangle defined
#define RVLMULCOV3VECT(A, x, y)                     \
    y[0] = A[0] * x[0] + A[1] * x[1] + A[2] * x[2]; \
    y[1] = A[1] * x[0] + A[4] * x[1] + A[5] * x[2]; \
    y[2] = A[2] * x[0] + A[5] * x[1] + A[8] * x[2];
// y = min(x(3x1))
#define RVL3DVECTORMIN(x, y)  \
    {                         \
        if (x[0] <= x[1])     \
        {                     \
            if (x[0] <= x[2]) \
                y = x[0];     \
            else              \
                y = x[2];     \
        }                     \
        else                  \
        {                     \
            if (x[1] <= x[2]) \
                y = x[1];     \
            else              \
                y = x[2];     \
        }                     \
    }
// y = A(3x3) * j-th column of B(3x3)
#define RVLMULMXCOL3(A, B, j, y)                            \
    y[0] = A[0] * B[j] + A[1] * B[3 + j] + A[2] * B[6 + j]; \
    y[1] = A[3] * B[j] + A[4] * B[3 + j] + A[5] * B[6 + j]; \
    y[2] = A[6] * B[j] + A[7] * B[3 + j] + A[8] * B[6 + j];
// C = A(3x3)*B(3x3)
#define RVLMXMUL3X3(A, B, C)                             \
    {                                                    \
        RVLMXEL(C, 3, 0, 0) = RVLMULROWCOL3(A, B, 0, 0); \
        RVLMXEL(C, 3, 0, 1) = RVLMULROWCOL3(A, B, 0, 1); \
        RVLMXEL(C, 3, 0, 2) = RVLMULROWCOL3(A, B, 0, 2); \
        RVLMXEL(C, 3, 1, 0) = RVLMULROWCOL3(A, B, 1, 0); \
        RVLMXEL(C, 3, 1, 1) = RVLMULROWCOL3(A, B, 1, 1); \
        RVLMXEL(C, 3, 1, 2) = RVLMULROWCOL3(A, B, 1, 2); \
        RVLMXEL(C, 3, 2, 0) = RVLMULROWCOL3(A, B, 2, 0); \
        RVLMXEL(C, 3, 2, 1) = RVLMULROWCOL3(A, B, 2, 1); \
        RVLMXEL(C, 3, 2, 2) = RVLMULROWCOL3(A, B, 2, 2); \
}
// C = A(3x3)*B'(3x3)
#define RVLMXMUL3X3T2(A, B, C)                           \
    {                                                    \
        RVLMXEL(C, 3, 0, 0) = RVLMULROWROW3(A, B, 0, 0); \
        RVLMXEL(C, 3, 0, 1) = RVLMULROWROW3(A, B, 0, 1); \
        RVLMXEL(C, 3, 0, 2) = RVLMULROWROW3(A, B, 0, 2); \
        RVLMXEL(C, 3, 1, 0) = RVLMULROWROW3(A, B, 1, 0); \
        RVLMXEL(C, 3, 1, 1) = RVLMULROWROW3(A, B, 1, 1); \
        RVLMXEL(C, 3, 1, 2) = RVLMULROWROW3(A, B, 1, 2); \
        RVLMXEL(C, 3, 2, 0) = RVLMULROWROW3(A, B, 2, 0); \
        RVLMXEL(C, 3, 2, 1) = RVLMULROWROW3(A, B, 2, 1); \
        RVLMXEL(C, 3, 2, 2) = RVLMULROWROW3(A, B, 2, 2); \
}
// C = A'(3x3)*B(3x3)
#define RVLMXMUL3X3T1(A, B, C)                           \
    {                                                    \
        RVLMXEL(C, 3, 0, 0) = RVLMULCOLCOL3(A, B, 0, 0); \
        RVLMXEL(C, 3, 0, 1) = RVLMULCOLCOL3(A, B, 0, 1); \
        RVLMXEL(C, 3, 0, 2) = RVLMULCOLCOL3(A, B, 0, 2); \
        RVLMXEL(C, 3, 1, 0) = RVLMULCOLCOL3(A, B, 1, 0); \
        RVLMXEL(C, 3, 1, 1) = RVLMULCOLCOL3(A, B, 1, 1); \
        RVLMXEL(C, 3, 1, 2) = RVLMULCOLCOL3(A, B, 1, 2); \
        RVLMXEL(C, 3, 2, 0) = RVLMULCOLCOL3(A, B, 2, 0); \
        RVLMXEL(C, 3, 2, 1) = RVLMULCOLCOL3(A, B, 2, 1); \
        RVLMXEL(C, 3, 2, 2) = RVLMULCOLCOL3(A, B, 2, 2); \
}
// Y = C(3x3)*J(3x3)'	(C is simmetric)
#define RVLMULCOV3MX3X3T(C, J, Y)                                      \
    {                                                                  \
        RVLMXEL(Y, 3, 0, 0) = C[0] * J[0] + C[1] * J[1] + C[2] * J[2]; \
        RVLMXEL(Y, 3, 0, 1) = C[0] * J[3] + C[1] * J[4] + C[2] * J[5]; \
        RVLMXEL(Y, 3, 0, 2) = C[0] * J[6] + C[1] * J[7] + C[2] * J[8]; \
        RVLMXEL(Y, 3, 1, 0) = C[1] * J[0] + C[4] * J[1] + C[5] * J[2]; \
        RVLMXEL(Y, 3, 1, 1) = C[1] * J[3] + C[4] * J[4] + C[5] * J[5]; \
        RVLMXEL(Y, 3, 1, 2) = C[1] * J[6] + C[4] * J[7] + C[5] * J[8]; \
        RVLMXEL(Y, 3, 2, 0) = C[2] * J[0] + C[5] * J[1] + C[8] * J[2]; \
        RVLMXEL(Y, 3, 2, 1) = C[2] * J[3] + C[5] * J[4] + C[8] * J[5]; \
        RVLMXEL(Y, 3, 2, 2) = C[2] * J[6] + C[5] * J[7] + C[8] * J[8]; \
}
#define RVLCOMPLETESIMMX3(A)         \
    {                                \
        A[3 * 1 + 0] = A[3 * 0 + 1]; \
        A[3 * 2 + 0] = A[3 * 0 + 2]; \
        A[3 * 2 + 1] = A[3 * 1 + 2]; \
}
// Y = A(3x3)*B(3x3)	(only diagonal + upper triangle are computed)
#define RVLMULMX3X3UT(A, B, Y)                                                                                  \
    {                                                                                                           \
        Y[3 * 0 + 0] = A[3 * 0 + 0] * B[3 * 0 + 0] + A[3 * 0 + 1] * B[3 * 1 + 0] + A[3 * 0 + 2] * B[3 * 2 + 0]; \
        Y[3 * 0 + 1] = A[3 * 1 + 0] * B[3 * 0 + 0] + A[3 * 1 + 1] * B[3 * 1 + 0] + A[3 * 1 + 2] * B[3 * 2 + 0]; \
        Y[3 * 0 + 2] = A[3 * 2 + 0] * B[3 * 0 + 0] + A[3 * 2 + 1] * B[3 * 1 + 0] + A[3 * 2 + 2] * B[3 * 2 + 0]; \
        Y[3 * 1 + 1] = A[3 * 1 + 0] * B[3 * 0 + 1] + A[3 * 1 + 1] * B[3 * 1 + 1] + A[3 * 1 + 2] * B[3 * 2 + 1]; \
        Y[3 * 1 + 2] = A[3 * 2 + 0] * B[3 * 0 + 1] + A[3 * 2 + 1] * B[3 * 1 + 1] + A[3 * 2 + 2] * B[3 * 2 + 1]; \
        Y[3 * 2 + 2] = A[3 * 2 + 0] * B[3 * 0 + 2] + A[3 * 2 + 1] * B[3 * 1 + 2] + A[3 * 2 + 2] * B[3 * 2 + 2]; \
}
// COut = J(3x3)*C(3x3)*J(3x3)'	(C is simmetric; only diagonal + upper triangle are computed)
#define RVLCOV3DTRANSF(CIn, J, COut, Tmp) \
    {                                     \
        RVLMULCOV3MX3X3T(CIn, J, Tmp)     \
        RVLMULMX3X3UT(J, Tmp, COut)       \
}
// x(3x1)'*y(3x1)
#define RVLDOTPRODUCT3(x, y) (x[0] * y[0] + x[1] * y[1] + x[2] * y[2])
#define RVLDOTPRODUCT3_64(x, y) ((int64)(x[0]) * (int64)(y[0]) + (int64)(x[1]) * (int64)(y[1]) + (int64)(x[2]) * (int64)(y[2]))
// z = x(3x1) x y(3x1)
#define RVLCROSSPRODUCT3(x, y, z)     \
    z[0] = x[1] * y[2] - x[2] * y[1]; \
    z[1] = x[2] * y[0] - x[0] * y[2]; \
    z[2] = x[0] * y[1] - x[1] * y[0];
// normalize vector x(3x1)
#define RVLNORM3(x, len)                  \
    {                                     \
        len = sqrt(RVLDOTPRODUCT3(x, x)); \
        RVLSCALE3VECTOR2(x, len, x);      \
    }
#define RVLSKEW(x, A)         \
    {                         \
        A[0 * 3 + 0] = 0.0;   \
        A[0 * 3 + 1] = -x[2]; \
        A[0 * 3 + 2] = x[1];  \
        A[1 * 3 + 0] = x[2];  \
        A[1 * 3 + 1] = 0.0;   \
        A[1 * 3 + 2] = -x[0]; \
        A[2 * 3 + 0] = -x[1]; \
        A[2 * 3 + 1] = x[0];  \
        A[2 * 3 + 2] = 0.0;   \
}
// A = x(3x1) * y(3x1)'
#define RVLMULVECT3VECT3T(x, y, A)  \
    {                               \
        A[3 * 0 + 0] = x[0] * y[0]; \
        A[3 * 0 + 1] = x[0] * y[1]; \
        A[3 * 0 + 2] = x[0] * y[2]; \
        A[3 * 1 + 0] = x[1] * y[0]; \
        A[3 * 1 + 1] = x[1] * y[1]; \
        A[3 * 1 + 2] = x[1] * y[2]; \
        A[3 * 2 + 0] = x[2] * y[0]; \
        A[3 * 2 + 1] = x[2] * y[1]; \
        A[3 * 2 + 2] = x[2] * y[2]; \
}
// x(3x1) * j-th column of A(3x3)
#define RVLMULVECTORCOL3(x, A, j) (x[0] * A[3 * 0 + j] + x[1] * A[3 * 1 + j] + x[2] * A[3 * 2 + j])
#define RVLCOVMX3BBVOLUME(C) (C[3 * 0 + 1] * (2.0 * C[3 * 0 + 2] * C[3 * 1 + 2] - C[3 * 2 + 2] * C[3 * 0 + 1]) - C[3 * 1 + 1] * C[3 * 0 + 2] * C[3 * 0 + 2] + C[3 * 0 + 0] * (C[3 * 1 + 1] * C[3 * 2 + 2] - C[3 * 1 + 2] * C[3 * 1 + 2]))
// invC = inv(C) (C is simmetric; only diagonal + upper triangle are computed)
#define RVLINVCOV3(C, invC, detC)                                                                                       \
    {                                                                                                                   \
        detC = 2.0 * C[5] * C[1] * C[2] - C[8] * C[1] * C[1] - C[4] * C[2] * C[2] - C[0] * (C[5] * C[5] - C[4] * C[8]); \
        invC[0] = (C[4] * C[8] - C[5] * C[5]) / detC;                                                                   \
        invC[1] = (C[2] * C[5] - C[1] * C[8]) / detC;                                                                   \
        invC[2] = (C[1] * C[5] - C[2] * C[4]) / detC;                                                                   \
        invC[4] = (C[0] * C[8] - C[2] * C[2]) / detC;                                                                   \
        invC[5] = (C[1] * C[2] - C[0] * C[5]) / detC;                                                                   \
        invC[8] = (C[0] * C[4] - C[1] * C[1]) / detC;                                                                   \
}
// return J(1x3)*C(3x3)*J(1x3)'
#define RVLCOV3DTRANSFTO1D(C, J) (C[0] * J[0] * J[0] + 2 * C[1] * J[0] * J[1] + 2 * C[2] * J[0] * J[2] + C[4] * J[1] * J[1] + 2 * C[5] * J[1] * J[2] + C[8] * J[2] * J[2])
#define RVLMIN(x, y) (x <= y ? (x) : (y))
#define RVLMAX(x, y) (x >= y ? (x) : (y))
#define RVLABS(x) (x >= 0.0 ? (x) : -(x))
// R = [1,  0,   0;
//		0, cs, -sn;
//		0, sn,  cs]
#define RVLROTX(cs, sn, R)         \
    {                              \
        RVLMXEL(R, 3, 0, 0) = 1.0; \
        RVLMXEL(R, 3, 0, 1) = 0.0; \
        RVLMXEL(R, 3, 0, 2) = 0.0; \
        RVLMXEL(R, 3, 1, 0) = 0.0; \
        RVLMXEL(R, 3, 1, 1) = cs;  \
        RVLMXEL(R, 3, 1, 2) = -sn; \
        RVLMXEL(R, 3, 2, 0) = 0.0; \
        RVLMXEL(R, 3, 2, 1) = sn;  \
        RVLMXEL(R, 3, 2, 2) = cs;  \
}
// R = [ cs, 0, sn;
//		  0, 1, 0;
//		-sn, 0, cs]
#define RVLROTY(cs, sn, R)         \
    {                              \
        RVLMXEL(R, 3, 0, 0) = cs;  \
        RVLMXEL(R, 3, 0, 1) = 0.0; \
        RVLMXEL(R, 3, 0, 2) = sn;  \
        RVLMXEL(R, 3, 1, 0) = 0.0; \
        RVLMXEL(R, 3, 1, 1) = 1.0; \
        RVLMXEL(R, 3, 1, 2) = 0.0; \
        RVLMXEL(R, 3, 2, 0) = -sn; \
        RVLMXEL(R, 3, 2, 1) = 0.0; \
        RVLMXEL(R, 3, 2, 2) = cs;  \
}
// R = [cs, -sn, 0;
//		sn,  cs, 0;
//		0,   0,  1]
#define RVLROTZ(cs, sn, R)         \
    {                              \
        RVLMXEL(R, 3, 0, 0) = cs;  \
        RVLMXEL(R, 3, 0, 1) = -sn; \
        RVLMXEL(R, 3, 0, 2) = 0.0; \
        RVLMXEL(R, 3, 1, 0) = sn;  \
        RVLMXEL(R, 3, 1, 1) = cs;  \
        RVLMXEL(R, 3, 1, 2) = 0.0; \
        RVLMXEL(R, 3, 2, 0) = 0.0; \
        RVLMXEL(R, 3, 2, 1) = 0.0; \
        RVLMXEL(R, 3, 2, 2) = 1.0; \
}
// V(3x1) = R(3x3) OX X ie[1 0 0]
#define RVLOX_X(R, V)                     \
    {                                     \
        V[0] = R[8] * R[4] - R[5] * R[7]; \
        V[1] = R[6] * R[5] - R[3] * R[8]; \
        V[2] = R[7] * R[3] - R[4] * R[6]; \
}
// V(3x1) = R(3x3) OX Z ie[0 0 1]
#define RVLOX_Z(R, V)                     \
    {                                     \
        V[0] = R[5] * R[1] - R[2] * R[4]; \
        V[1] = R[3] * R[2] - R[0] * R[5]; \
        V[2] = R[4] * R[0] - R[1] * R[3]; \
}
// C(3x3) = A(3x1) * B'(3x1)
#define RVLVECMUL3X1T2(A, B, C)            \
    {                                      \
        RVLMXEL(C, 3, 0, 0) = A[0] * B[0]; \
        RVLMXEL(C, 3, 0, 1) = A[0] * B[1]; \
        RVLMXEL(C, 3, 0, 2) = A[0] * B[2]; \
        RVLMXEL(C, 3, 1, 0) = A[1] * B[0]; \
        RVLMXEL(C, 3, 1, 1) = A[1] * B[1]; \
        RVLMXEL(C, 3, 1, 2) = A[1] * B[2]; \
        RVLMXEL(C, 3, 2, 0) = A[2] * B[0]; \
        RVLMXEL(C, 3, 2, 1) = A[2] * B[1]; \
        RVLMXEL(C, 3, 2, 2) = A[2] * B[2]; \
}
// C(3x3) = x(3x1)*x(3x1)'		(only diagonal + upper triangle are computed)
#define RVLVECTCOV3(x, C)     \
    {                         \
        C[0] = (x[0] * x[0]); \
        C[1] = (x[0] * x[1]); \
        C[2] = (x[0] * x[2]); \
        C[4] = (x[1] * x[1]); \
        C[5] = (x[1] * x[2]); \
        C[8] = (x[2] * x[2]); \
    }
// C(2x2) = C(2x2) + x(2x1)*x(2x1)'		(only diagonal + upper triangle are computed)
// M(2x1) = M(2x1) + x(2x1)
#define RVLMOMENTS2UPDATE(x, M, C, n) \
    {                                 \
        n++;                          \
        M[0] += x[0];                 \
        M[1] += x[1];                 \
        C[0] += (x[0] * x[0]);        \
        C[1] += (x[0] * x[1]);        \
        C[3] += (x[1] * x[1]);        \
}
// C(3x3) = C(3x3) + x(3x1)*x(3x1)'		(only diagonal + upper triangle are computed)
// M(3x1) = M(3x1) + x(3x1)
#define RVLMOMENTS3UPDATE(x, M, C, n) \
    {                                 \
        n++;                          \
        M[0] += x[0];                 \
        M[1] += x[1];                 \
        M[2] += x[2];                 \
        C[0] += (x[0] * x[0]);        \
        C[1] += (x[0] * x[1]);        \
        C[2] += (x[0] * x[2]);        \
        C[4] += (x[1] * x[1]);        \
        C[5] += (x[1] * x[2]);        \
        C[8] += (x[2] * x[2]);        \
}
// pTgt = R * pSrc + t
#define RVLTRANSF3(pSrc, R, t, pTgt)   \
    {                                  \
        RVLMULMX3X3VECT(R, pSrc, pTgt) \
        RVLSUM3VECTORS(pTgt, t, pTgt)  \
}
// pTgt = R' * (pSrc - t)
#define RVLINVTRANSF3(pSrc, R, t, pTgt, tmp3x1) \
    {                                           \
        RVLDIF3VECTORS(pSrc, t, tmp3x1);        \
        RVLMULMX3X3TVECT(R, tmp3x1, pTgt);      \
}
// T(R, t) = T(R1, t1) * T(R2, t2)
#define RVLCOMPTRANSF3D(R1, t1, R2, t2, R, t) \
    {                                         \
        RVLMXMUL3X3(R1, R2, R)                \
        RVLMULMX3X3VECT(R1, t2, t)            \
        RVLSUM3VECTORS(t, t1, t)              \
}
// pTgt = diag(s) * R * pSrc + t
#define RVLATRANSF3(pSrc, s, R, t, pTgt, V3Tmp) \
    {                                           \
        RVLSCALE3VECTOR3(pSrc, s, V3Tmp)        \
        RVLMULMX3X3VECT(R, V3Tmp, pTgt)         \
        RVLSUM3VECTORS(pTgt, t, pTgt)           \
}
// R = R1 * R2	(rotations around z-axis)
#define RVLCOMPROT3D3DOF(R1, R2, R)           \
    {                                         \
        R[0] = R1[0] * R2[0] + R1[1] * R2[3]; \
        R[1] = R1[0] * R2[1] + R1[1] * R2[4]; \
        R[2] = 0.0;                           \
        R[3] = R1[3] * R2[0] + R1[4] * R2[3]; \
        R[4] = R1[3] * R2[1] + R1[4] * R2[4]; \
        R[5] = 0.0;                           \
        R[6] = 0.0;                           \
        R[7] = 0.0;                           \
        R[8] = 1.0;                           \
}
// T(R, t) = T(R1, t1) * T(R2, t2)	(3DOF transformations)
#define RVLCOMPTRANSF3D3DOF(R1, t1, R2, t2, R, t)     \
    {                                                 \
        RVLCOMPROT3D3DOF(R1, R2, R)                   \
        t[0] = R1[0] * t2[0] + R1[1] * t2[1] + t1[0]; \
        t[1] = R1[3] * t2[0] + R1[4] * t2[1] + t1[1]; \
        t[2] = 0.0;                                   \
}
// T(R, t) = inv(T(R1, T1)) * T(R2, T2)
#define RVLCOMPTRANSF3DWITHINV(R1, t1, R2, t2, R, t, tmp3x1) \
    {                                                        \
        RVLMXMUL3X3T1(R1, R2, R)                             \
        RVLDIF3VECTORS(t2, t1, tmp3x1)                       \
        RVLMULMX3X3TVECT(R1, tmp3x1, t)                      \
}
// T(RTgt, tTgt) = inv(T(RSrc, tSrc))
#define RVLINVTRANSF3D(RSrc, tSrc, RTgt, tTgt) \
    {                                          \
        RVLCOPYMX3X3T(RSrc, RTgt)              \
        RVLINVTRANSL(RSrc, tSrc, tTgt)         \
}
// T(RTgt, tTgt) = inv(T(RSrc, tSrc))	(3DOF transformations)
#define RVLINVTRANSF3D3DOF(RSrc, tSrc, RTgt, tTgt)        \
    {                                                     \
        RTgt[0] = RSrc[0];                                \
        RTgt[1] = RSrc[3];                                \
        RTgt[2] = 0.0;                                    \
        RTgt[3] = RSrc[1];                                \
        RTgt[4] = RSrc[4];                                \
        RTgt[5] = 0.0;                                    \
        RTgt[6] = 0.0;                                    \
        RTgt[7] = 0.0;                                    \
        RTgt[8] = 1.0;                                    \
        tTgt[0] = -RSrc[0] * tSrc[0] - RSrc[3] * tSrc[1]; \
        tTgt[1] = RSrc[3] * tSrc[0] - RSrc[0] * tSrc[1];  \
        tTgt[2] = 0.0;                                    \
}
#define RVLHTRANSFMX(R, t, T)        \
    {                                \
        T[0] = R[0];                 \
        T[1] = R[1];                 \
        T[2] = R[2];                 \
        T[4] = R[3];                 \
        T[5] = R[4];                 \
        T[6] = R[5];                 \
        T[8] = R[6];                 \
        T[9] = R[7];                 \
        T[10] = R[8];                \
        T[3] = t[0];                 \
        T[7] = t[1];                 \
        T[11] = t[2];                \
        T[12] = T[13] = T[14] = 0.0; \
        T[15] = 1.0;                 \
}
#define RVLHTRANSFMXDECOMP(T, R, t) \
    {                               \
        R[0] = T[0];                \
        R[1] = T[1];                \
        R[2] = T[2];                \
        R[3] = T[4];                \
        R[4] = T[5];                \
        R[5] = T[6];                \
        R[6] = T[8];                \
        R[7] = T[9];                \
        R[8] = T[10];               \
        t[0] = T[3];                \
        t[1] = T[7];                \
        t[2] = T[11];               \
}

#define RVLHTRANSFMXDECOMP_COLMAY(T, R, t) \
    {                                      \
        R[0] = T[0];                       \
        R[1] = T[4];                       \
        R[2] = T[8];                       \
        R[3] = T[1];                       \
        R[4] = T[5];                       \
        R[5] = T[9];                       \
        R[6] = T[2];                       \
        R[7] = T[6];                       \
        R[8] = T[10];                      \
        t[0] = T[12];                      \
        t[1] = T[13];                      \
        t[2] = T[14];                      \
}
// Compute s and RTgt such that RSrc = s * RTgt.
#define RVLEXTRACTSCALEFROMROT(RSrc, s, RTgt) \
    {                                         \
        s = sqrt(RVLDOTPRODUCT3(RSrc, RSrc)); \
        RVLSCALEMX3X32(RSrc, s, RTgt)         \
    }

//// Compute vector Y orthogonal to X
// #define RVLORTHOGONAL3(X, Y, i, j, k, tmp3x1, fTmp)\
//{\
//	tmp3x1[0] = RVLABS(X[0]);\
//	tmp3x1[1] = RVLABS(X[1]);\
//	tmp3x1[2] = RVLABS(X[2]);\
//	i = (tmp3x1[0] > tmp3x1[1] ? 0 : 1);\
//	if(tmp3x1[2] > tmp3x1[i])\
//		i = 2;\
//	j = (i + 1) % 3;\
//	k = (i + 2) % 3;\
//	Y[i] = -X[j];\
//	Y[j] = X[i];\
//	Y[k] = 0.0;\
//	fTmp = sqrt(Y[j] * Y[j] + Y[i] * Y[i]);\
//	RVLSCALE3VECTOR2(Y, fTmp, Y)\
//}
//  Compute vector Y orthogonal to X
#define RVLORTHOGONAL3(X, Y, i, j, k, fTmp)        \
    {                                              \
        i = (RVLABS(X[0]) < RVLABS(X[1]) ? 0 : 1); \
        j = (i + 1) % 3;                           \
        k = (i + 2) % 3;                           \
        Y[j] = -X[k];                              \
        Y[k] = X[j];                               \
        Y[i] = 0.0;                                \
        fTmp = sqrt(Y[j] * Y[j] + Y[k] * Y[k]);    \
        RVLSCALE3VECTOR2(Y, fTmp, Y)               \
}
// Tgt = Src(2x2)
#define RVLCOPYMX2X2(Src, Tgt) \
    Tgt[0] = Src[0];           \
    Tgt[1] = Src[1];           \
    Tgt[2] = Src[2];           \
    Tgt[3] = Src[3];
// C = A(2x2)*B(2x2)
#define RVLMXMUL2X2(A, B, C)                                                                                         \
    {                                                                                                                \
        RVLMXEL(C, 2, 0, 0) = RVLMXEL(A, 2, 0, 0) * RVLMXEL(B, 2, 0, 0) + RVLMXEL(A, 2, 0, 1) * RVLMXEL(B, 2, 1, 0); \
        RVLMXEL(C, 2, 0, 1) = RVLMXEL(A, 2, 0, 0) * RVLMXEL(B, 2, 0, 1) + RVLMXEL(A, 2, 0, 1) * RVLMXEL(B, 2, 1, 1); \
        RVLMXEL(C, 2, 1, 0) = RVLMXEL(A, 2, 1, 0) * RVLMXEL(B, 2, 0, 0) + RVLMXEL(A, 2, 1, 1) * RVLMXEL(B, 2, 1, 0); \
        RVLMXEL(C, 2, 1, 1) = RVLMXEL(A, 2, 1, 0) * RVLMXEL(B, 2, 0, 1) + RVLMXEL(A, 2, 1, 1) * RVLMXEL(B, 2, 1, 1); \
}
// y = det(C(2x2)) (C is simmetric)
#define RVLDET2(C) (C[0] * C[3] - C[1] * C[1])
// y = x(2x1)' * inv(C(2x2)) * x (C is simmetric)
#define RVLMAHDIST2(x, C, detC) ((C[3] * x[0] * x[0] - 2.0 * C[1] * x[0] * x[1] + C[0] * x[1] * x[1]) / detC)
// COut = J(2x2)*C(2x2)*J(2x2)'	(C is simmetric; only diagonal + upper triangle are computed)
#define RVLCOV2DTRANSF(C, J, COut)                                                         \
    {                                                                                      \
        COut[0] = C[0] * J[0] * J[0] + 2 * C[1] * J[0] * J[1] + C[3] * J[1] * J[1];        \
        COut[1] = J[2] * (C[0] * J[0] + C[1] * J[1]) + J[3] * (C[1] * J[0] + C[3] * J[1]); \
        COut[3] = C[0] * J[2] * J[2] + 2 * C[1] * J[2] * J[3] + C[3] * J[3] * J[3];        \
}
// y = A(2x2) * x(2x1), where A is a simetric matrix with only diagonal + upper triangle defined
#define RVLMULCOV2VECT(A, x, y)       \
    y[0] = A[0] * x[0] + A[1] * x[1]; \
    y[1] = A[1] * x[0] + A[3] * x[1];
// invC(2x2) = inv(C(2x2)) (C is simmetric; only diagonal + upper triangle are computed)
#define RVLINVCOV2(C, invC, detC) \
    {                             \
        invC[0] = C[3] / detC;    \
        invC[1] = -C[1] / detC;   \
        invC[3] = C[0] / detC;    \
}
#define RVLCONVTOINT3(Src, Tgt) \
    Tgt[0] = (int)Src[0];       \
    Tgt[1] = (int)Src[1];       \
    Tgt[2] = (int)Src[2];
#define RVLCONVTOUCHAR3(Src, Tgt)   \
    Tgt[0] = (unsigned char)Src[0]; \
    Tgt[1] = (unsigned char)Src[1]; \
    Tgt[2] = (unsigned char)Src[2];
#define RVLSORT3ASCEND(Vect3, idx, tmp)    \
    {                                      \
        if (Vect3[0] <= Vect3[1])          \
        {                                  \
            idx[0] = 0;                    \
            idx[1] = 1;                    \
        }                                  \
        else                               \
        {                                  \
            idx[0] = 1;                    \
            idx[1] = 0;                    \
        }                                  \
        if (Vect3[idx[0]] > Vect3[2])      \
        {                                  \
            idx[2] = idx[0];               \
            idx[0] = 2;                    \
        }                                  \
        else                               \
            idx[2] = 2;                    \
        if (Vect3[idx[1]] > Vect3[idx[2]]) \
        {                                  \
            tmp = idx[1];                  \
            idx[1] = idx[2];               \
            idx[2] = tmp;                  \
        }                                  \
}
// AT(3x3) = A(3x3)' - Vidovic
#define RVLTRASPOSE3X3(A, AT) \
    {                         \
        AT[0] = A[0];         \
        AT[1] = A[3];         \
        AT[2] = A[6];         \
        AT[3] = A[1];         \
        AT[4] = A[4];         \
        AT[5] = A[7];         \
        AT[6] = A[2];         \
        AT[7] = A[5];         \
        AT[8] = A[8];         \
}
#define RVLQUATERNIONTOROT(Q, R)                                              \
    {                                                                         \
        R[0 * 3 + 0] = Q[0] * Q[0] + Q[1] * Q[1] - Q[2] * Q[2] - Q[3] * Q[3]; \
        R[1 * 3 + 0] = 2.0 * Q[1] * Q[2] + 2.0 * Q[0] * Q[3];                 \
        R[2 * 3 + 0] = 2.0 * Q[1] * Q[3] - 2.0 * Q[0] * Q[2];                 \
        R[0 * 3 + 1] = 2.0 * Q[1] * Q[2] - 2.0 * Q[0] * Q[3];                 \
        R[1 * 3 + 1] = Q[0] * Q[0] - Q[1] * Q[1] + Q[2] * Q[2] - Q[3] * Q[3]; \
        R[2 * 3 + 1] = 2.0 * Q[2] * Q[3] + 2.0 * Q[0] * Q[1];                 \
        R[0 * 3 + 2] = 2.0 * Q[1] * Q[3] + 2.0 * Q[0] * Q[2];                 \
        R[1 * 3 + 2] = 2.0 * Q[2] * Q[3] - 2.0 * Q[0] * Q[1];                 \
        R[2 * 3 + 2] = Q[0] * Q[0] - Q[1] * Q[1] - Q[2] * Q[2] + Q[3] * Q[3]; \
}
#define RVLROTDIFF(R) (0.5 * (R[0] + R[4] + R[8] - 1.0))

template <typename Type>
struct Moments2D
{
    int n;
    Type S[2], S2[4];
};

template <typename Type>
inline void InitMoments2D(Moments2D<Type> &moments)
{
    moments.n = 0;
    moments.S[0] = moments.S[1] = 0.0f;
    moments.S2[0] = moments.S2[1] = moments.S2[2] = moments.S2[3] = 0.0f;
}

template <typename Type>
inline void UpdateMoments2D(Moments2D<Type> &moments, Type *P)
{
    RVLMOMENTS2UPDATE(P, moments.S, moments.S2, moments.n);
}

template <class Type>
void GetCovMatrix2(
    Moments2D<Type> *pMoments,
    Type *C,
    Type *M)
{
    Type fn = (Type)(pMoments->n);
    M[0] = pMoments->S[0] / fn;
    M[1] = pMoments->S[1] / fn;
    C[0] = pMoments->S2[0] / fn - M[0] * M[0];
    C[1] = pMoments->S2[1] / fn - M[0] * M[1];
    C[2] = C[1];
    C[3] = pMoments->S2[3] / fn - M[1] * M[1];
}

template <typename Type>
struct Moments
{
	int n;
	Type S[3], S2[9];
};

template <typename Type>
inline void InitMoments(Moments<Type> &moments)
{
	moments.n = 0;
	RVLNULL3VECTOR(moments.S);
	RVLNULLMX3X3(moments.S2);
}

template <typename Type>
inline void UpdateMoments(Moments<Type> &moments, Type *P)
{
	RVLMOMENTS3UPDATE(P, moments.S, moments.S2, moments.n);
}

template <typename Type>
inline void SumMoments(
	Moments<Type> momentsSrc1, 
	Moments<Type> momentsSrc2, 
	Moments<Type> &momentsTgt)
{
	momentsTgt.n = momentsSrc1.n + momentsSrc2.n;
	RVLSUMMX3X3(momentsSrc1.S2, momentsSrc2.S2, momentsTgt.S2);
	RVLSUM3VECTORS(momentsSrc1.S, momentsSrc2.S, momentsTgt.S);
}

template <class Type>
void GetCovMatrix3(
	Moments<Type> *pMoments,
	Type *C,
	Type *M)
{
	Type fn = (Type)(pMoments->n);
	M[0] = pMoments->S[0] / fn;
	M[1] = pMoments->S[1] / fn;
	M[2] = pMoments->S[2] / fn;
	C[0] = pMoments->S2[0] / fn - M[0] * M[0];
	C[1] = pMoments->S2[1] / fn - M[0] * M[1];
	C[2] = pMoments->S2[2] / fn - M[0] * M[2];
	C[3] = C[1];
	C[4] = pMoments->S2[4] / fn - M[1] * M[1];
	C[5] = pMoments->S2[5] / fn - M[1] * M[2];
	C[6] = C[2];
	C[7] = C[5];
	C[8] = pMoments->S2[8] / fn - M[2] * M[2];
}

template <typename Type>
inline void LinePlaneIntersection(
	Type *P1,
	Type *P2,
	Type *N,
	Type d,
	Type *PIS)
{
	Type dP[3];

	RVLDIF3VECTORS(P2, P1, dP);

	Type s = (d - RVLDOTPRODUCT3(N, P1)) / (RVLDOTPRODUCT3(N, dP));

	RVLSCALE3VECTOR(dP, s, PIS);
	RVLSUM3VECTORS(PIS, P1, PIS);
}

namespace RVL
{
	struct Pose3D
	{
		float R[9];
		float t[3];
	};

    template <typename Type>
    struct Box
	{
		Type minx;
		Type maxx;
		Type miny;
		Type maxy;
		Type minz;
		Type maxz;
	};

    template <typename Type>
    struct Sphere
	{
		Type center[3];
		Type radius;
	};

	template <typename T>
	void ExpandBox(Box<T> *pBox, T extension)
	{
		pBox->minx -= extension;
		pBox->maxx += extension;
		pBox->miny -= extension;
		pBox->maxy += extension;
		pBox->minz -= extension;
		pBox->maxz += extension;
	}

    template <typename Type>
    void PrintMatrix(FILE *fp, Type *A, int n, int m)
	{
		Type *pA = A;

		int i, j;

		for (i = 0; i < n; i++)
		{
			for (j = 0; j < m; j++, pA++)
				fprintf(fp, "%f\t", *pA);

			fprintf(fp, "\n");
		}
	}

    template <typename Type>
    void AngleAxisToRot(Type *k, Type q, Type *R)
	{
		Type cq = cos(q);
		Type sq = sin(q);
		Type cqcomp = 1.0 - cq;
		Type kxy = k[0] * k[1] * cqcomp;
		Type kyz = k[1] * k[2] * cqcomp;
		Type kzx = k[2] * k[0] * cqcomp;

		R[0] = k[0] * k[0] * cqcomp + cq;
		R[1] = kxy - k[2] * sq;
		R[2] = kzx + k[1] * sq;
		R[3] = kxy + k[2] * sq;
		R[4] = k[1] * k[1] * cqcomp + cq;
		R[5] = kyz - k[0] * sq;
		R[6] = kzx - k[1] * sq;
		R[7] = kyz + k[0] * sq;
		R[8] = k[2] * k[2] * cqcomp + cq;
	}

#define RVLCREATE3DTRANSF(R, t, T)   \
    {                                \
        T[0] = R[0];                 \
        T[1] = R[1];                 \
        T[2] = R[2];                 \
        T[4] = R[3];                 \
        T[5] = R[4];                 \
        T[6] = R[5];                 \
        T[8] = R[6];                 \
        T[9] = R[7];                 \
        T[10] = R[8];                \
        T[3] = t[0];                 \
        T[7] = t[1];                 \
        T[11] = t[2];                \
        T[12] = T[13] = T[14] = 0.0; \
        T[15] = 1.0;                 \
	}

    template <typename T>
    struct Vector3
	{
		T Element[3];
	};

    template <typename T>
    struct Matrix3
	{
		T Element[9];
	};

    template <typename T1, typename T2>
    struct Correspondence
	{
		T1 item1;
		T2 item2;
	};

	template <typename T>
	void InitBoundingBox(Box<T> *pBox, T *P)
	{
		pBox->minx = pBox->maxx = P[0];
		pBox->miny = pBox->maxy = P[1];
		pBox->minz = pBox->maxz = P[2];
	}

	template <typename T>
	void UpdateBoundingBox(Box<T> *pBox, T *P)
	{
		if (P[0] < pBox->minx)
			pBox->minx = P[0];
		else if (P[0] > pBox->maxx)
			pBox->maxx = P[0];

		if (P[1] < pBox->miny)
			pBox->miny = P[1];
		else if (P[1] > pBox->maxy)
			pBox->maxy = P[1];

		if (P[2] < pBox->minz)
			pBox->minz = P[2];
		else if (P[2] > pBox->maxz)
			pBox->maxz = P[2];
	}

	template <typename T>
	bool InBoundingBox(Box<T> *pBox, T *P)
	{
		return (P[0] >= pBox->minx && P[0] <= pBox->maxx &&
			P[1] >= pBox->miny && P[1] <= pBox->maxy &&
			P[2] >= pBox->minz && P[2] <= pBox->maxz);
	}

	template <typename T>
	bool BoxIntersection(
		Box<T> *pBoxSrc1,
		Box<T> *pBoxSrc2,
		Box<T> *pBoxTgt)
	{
		pBoxTgt->minx = RVLMAX(pBoxSrc1->minx, pBoxSrc2->minx);
		pBoxTgt->maxx = RVLMIN(pBoxSrc1->maxx, pBoxSrc2->maxx);

		if (pBoxTgt->minx >= pBoxTgt->maxx)
			return false;

		pBoxTgt->miny = RVLMAX(pBoxSrc1->miny, pBoxSrc2->miny);
		pBoxTgt->maxy = RVLMIN(pBoxSrc1->maxy, pBoxSrc2->maxy);

		if (pBoxTgt->miny >= pBoxTgt->maxy)
			return false;

		pBoxTgt->minz = RVLMAX(pBoxSrc1->minz, pBoxSrc2->minz);
		pBoxTgt->maxz = RVLMIN(pBoxSrc1->maxz, pBoxSrc2->maxz);

		if (pBoxTgt->minz >= pBoxTgt->maxz)
			return false;

		return true;
	}

	template <typename T>
	void BoxSize(
		Box<T> *pBox,
		T &a,
		T &b,
		T &c)
	{
		a = pBox->maxx - pBox->minx;
		b = pBox->maxy - pBox->miny;
		c = pBox->maxz - pBox->minz;
	}

	template <typename T>
	T BoxSize(Box<T> *pBox)
	{
		T a, b, c;

		BoxSize(pBox, a, b, c);

		T tmp = RVLMAX(a, b);

		return RVLMAX(tmp, c);
	}

	template <typename T>
	T BoxVolume(Box<T> *pBox)
	{
		return (pBox->maxx - pBox->minx) * (pBox->maxy - pBox->miny) * (pBox->maxz - pBox->minz);
	}

	template <typename T>
	void BoxCenter(
		Box<T> *pBox,
		T *P)
	{
		P[0] = 0.5f * (pBox->minx + pBox->maxx);
		P[1] = 0.5f * (pBox->miny + pBox->maxy);
		P[2] = 0.5f * (pBox->minz + pBox->maxz);
	}

	template <typename T>
	void BoxVertices(
		Box<T> *pBox,
		T *P)
	{
		T *P_;

        P_ = P + 0 * 3;
        RVLSET3VECTOR(P_, pBox->minx, pBox->miny, pBox->minz);
        P_ = P + 1 * 3;
        RVLSET3VECTOR(P_, pBox->maxx, pBox->miny, pBox->minz);
        P_ = P + 2 * 3;
        RVLSET3VECTOR(P_, pBox->minx, pBox->maxy, pBox->minz);
        P_ = P + 3 * 3;
        RVLSET3VECTOR(P_, pBox->maxx, pBox->maxy, pBox->minz);
        P_ = P + 4 * 3;
        RVLSET3VECTOR(P_, pBox->minx, pBox->miny, pBox->maxz);
        P_ = P + 5 * 3;
        RVLSET3VECTOR(P_, pBox->maxx, pBox->miny, pBox->maxz);
        P_ = P + 6 * 3;
        RVLSET3VECTOR(P_, pBox->minx, pBox->maxy, pBox->maxz);
        P_ = P + 7 * 3;
        RVLSET3VECTOR(P_, pBox->maxx, pBox->maxy, pBox->maxz);
	}

	template <typename T>
	T ComparePosesUsingReferenceVertices(
		T *R,
		T *t,
		Array2D<T> PArray)
	{
		T P_[3];

		T E = 0.0;

		int i;
		T *P;
		T dP[3];

		for (i = 0; i < PArray.h; i++)
		{
			P = PArray.Element + 3 * i;

			RVLTRANSF3(P, R, t, P_);

			RVLDIF3VECTORS(P, P_, dP);

			E += RVLDOTPRODUCT3(dP, dP);
		}
		
		E /= ((T)(PArray.h));

		return E;
	}
}
