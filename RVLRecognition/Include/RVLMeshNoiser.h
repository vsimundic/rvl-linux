//#pragma once

namespace RVL
{

	class MeshNoiser
	{
	public:
		MeshNoiser();
		~MeshNoiser();
		void SetParam(float pointStdDev, float normalStdDev);
		void AddWhiteNoise(Mesh *pMesh);


	private:
		float	pointStdDev;
		float	normalStdDev;

	};
}