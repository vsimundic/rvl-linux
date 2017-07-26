#define RVL_NONE				0xffff
#define RVL_INDEX				0x3fff
#define RVL_OPPOSITE			0x8000
#define RVL_RES_OK				0
#define RVL_MAX_FILE_NAME_LEN	200

#ifndef BOOL
#define BOOL int
#endif
#ifndef BYTE
#define BYTE unsigned char
#endif
#ifndef WORD
#ifdef RVL64BIT
#define WORD unsigned int
#else
#define WORD unsigned short int
#endif
#endif
#ifndef DWORD
//#ifdef RVL64BIT
//#define DWORD unsigned long int
//#else
#ifdef RVL64BIT
#define DWORD unsigned long long
#else
#define DWORD unsigned int
#endif
#endif

#define PI			3.14159265358979
#define COS20		0.94	
#define COS20_2		(COS20 * COS20)
#define COS20_2_NRM	226
#define DEG2RAD		(PI / 180.0)
#define RAD2DEG		(180.0 / PI)
#define COS45		0.707
#define RAD2NRM		(120.0 / PI)
#define NRM2RAD		(PI / 120.0)
#define DEG2NRM		(120.0 / 180.0)
#define NRM2DEG		(180.0 / 120.0)
#define SQRT2		1.4142135623731
#define SQRT0_5		0.707106781186548
#define TRUE		1
#define FALSE		0
#define TAN22p5		0.41421356237309505
#define CHI2INV_1D_P99	6.63489660102123
#define CHI2INV_2D_P99	9.21034037197618
#define CHI2INV_3D_P99	11.3448667301444
#define RVLLN2			0.69314718055994530941723212145818
#define RVLLNPI			1.1447298858494001741434273513531
#define RVLLN2PI		1.8378770664093454835606594728112
#define RVLLN4PI		(RVLLN2 + RVLLN2PI)
#define RVLLN1p01		0.00995033085316808284821535754426


