#pragma once

namespace RVL
{
	class DisplayVector
	{
	public:
		DisplayVector();
		virtual ~DisplayVector();

	public:
		unsigned char lineColor[3];
		unsigned char pointColor[3];
		int lineWidth;
		BYTE pointType;
		bool bClosed;
		QList<QLIST::Entry<CvPoint>> pointList;
		DisplayVector *pNext;
	};
}

