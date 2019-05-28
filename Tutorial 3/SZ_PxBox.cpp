#pragma once
#include "SZ_PxBox.h"
namespace PhysicsEngine
{
	SZ_PxBox::SZ_PxBox(const PxTransform & pose, PxVec3 dimensions, PxReal density) : DynamicActor(pose)
	{
		CreateShape(PxBoxGeometry(dimensions), density);
	}
	void SZ_PxBox::Render()
	{
		//printf("SZ_PxBox::Render()");
		//Get the position of this object
		PxTransform pose = ((PxRigidBody*)Get())->getGlobalPose();
		PxMat44 shapePose(pose);
		//move the opengl matrix to match the physx object
		glPushMatrix();
		glMultMatrixf((float*)&shapePose);

		//currently, renderer is at center of mass of object, use standard opengl
		//glutSolidSphere(1.5, 6, 6);
		glutWireSphere(1.5, 6, 6);
		//pop opengl matrix back to before
		glPopMatrix();
	}
}