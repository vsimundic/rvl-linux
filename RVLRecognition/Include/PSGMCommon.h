namespace RVL
{
	namespace RECOG
	{
		namespace PSGM_
		{
			struct Plane
			{
				float N[3];
				float d;
			};

			struct Tangent
			{
				float N[3];
				float V[3];
				float d;
				float len;
				int iVertex[2];
				bool bMerged;
			};
		}
	}
}