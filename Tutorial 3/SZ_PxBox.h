#pragma once
#include "PhysicsEngine.h"
#include <iostream>
#include <iomanip>
#include "Extras/Renderer.h"
#include <vector>

//custom invisible box class derived from lecture slides

namespace PhysicsEngine
{
	class SZ_PxBox : public DynamicActor
	{
	public:
		SZ_PxBox(const PxTransform& pose, PxVec3 dimensions, PxReal density);
		void Render();
	};
}
