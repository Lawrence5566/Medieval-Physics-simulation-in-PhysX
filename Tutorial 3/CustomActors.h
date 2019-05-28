#pragma once

#include "PhysicsEngine.h"
#include "BasicActors.h"
#include <iostream>
#include <iomanip>

namespace PhysicsEngine
{
	class CustomMaterials {
	public:
		PxMaterial* ballMat = CreateMaterial(0.8f, 0.7f, 0.80f);	//static, dynamic, restitution (leather), high static and dynamic friction but high restitution (very bouncy)s
		PxMaterial* woodMat = CreateMaterial(0.3f, 0.35f, 0.6f);	//slides easily, absorbs medium amount of force - slightly less bouncy than steal according to the internet...
		PxMaterial* steelMat = CreateMaterial(0.4f, 0.47f, 0.7f);	//strong, slides easily, bouncy but not as much as ball
		PxMaterial* grassMat = CreateMaterial(0.85f, 0.8f, 0.0f);	//high friction, massive force absorbed as we are modelling this as the earth, since the earth does not bounce when you throw and object at its
	};
	

	//custom actors//

	class Ellipsoid : public ConvexMesh {
	//using sphere c#:
	//http://wiki.unity3d.com/index.php/ProceduralPrimitives#C.23_-_Sphere

	public:
		vector<PxVec3> calculateVerts(float scale, int seg_1, int seg_2) { 
			vector<PxVec3> verts;

			float radius = 1.0f*scale;
			float lradius = 0.65f*scale; //different radius for x & z component

			int nbLong = seg_1; //segment numbers
			int nbLat = seg_2;

			float _pi = PxPi;
			float _2pi = _pi * 2.f;

			verts.push_back(PxVec3(0, radius, 0)); //top point
			for (int lat = 0; lat < nbLat; lat++)
			{
				float a1 = _pi * (float)(lat + 1) / (nbLat + 1);
				float sin1 = sinf(a1);
				float cos1 = cosf(a1);

				for (int lon = 0; lon <= nbLong; lon++)
				{
					float a2 = _2pi * (float)(lon == nbLong ? 0 : lon) / nbLong;
					float sin2 = sinf(a2);
					float cos2 = cosf(a2);

					//vertices[lon + lat * (nbLong + 1) + 1] = new Vector3(sin1 * cos2, cos1, sin1 * sin2) * radius;

					verts.push_back(PxVec3(sin1 * cos2 * lradius, cos1 * radius, sin1 * sin2 * lradius));

					//cout << verts[verts.size()-1].x << "," << verts[verts.size() - 1].y << "," << verts[verts.size() - 1].z << endl;
				}
			}
			verts.push_back(PxVec3(0, -radius, 0)); //bottom point

			return verts;
		}

	public:
		Ellipsoid(float scale = 1.0f, int seg_1 = 12, int seg_2 = 8, PxTransform pose = PxTransform(PxIdentity), PxReal density = 1.f) :
			ConvexMesh(calculateVerts(scale, seg_1, seg_2), pose, density)
		{

		}

	};

	//cylinder = new Cylinder(1.f, .5f, 6, PxTransform(PxVec3(0.f, 2.0f, 0.f)), 1.f);
	class Cylinder : public ConvexMesh
	{
		vector<PxVec3> calculateCirlceVerts(float cx, float cy, float cz, float r, int seg_num) { //cx & cy = center x, center y, cz = depth
			vector<PxVec3> verts;

			for (int i = 0; i < seg_num; i++) {
				float theta = 2.0f * 3.1415926f * float(i) / float(seg_num);//get the current angle

				float x = r * cosf(theta);//calculate the x component
				float y = r * sinf(theta);//calculate the y component

				verts.push_back(PxVec3(x + cx, y + cy, -cz / 2)); //final front vertex (split depth in 2 to center shape properly)
				verts.push_back(PxVec3(x + cx, y + cy, cz / 2)); // corresponding depth vertex
			}

			return verts;
		}

	public:
		Cylinder(float length = 1.f, float radius = 1.f, int seg_num = 6, PxTransform pose = PxTransform(PxIdentity), PxReal density = 1.f) :
			ConvexMesh(calculateCirlceVerts(0.f, 0.f, length, radius, seg_num), pose, density)
		{

		}

	};

	class PrismaticJoint : public Joint
	{
	public:
		PrismaticJoint(Actor* actor0, const PxTransform& localFrame0, Actor* actor1, const PxTransform& localFrame1)
		{
			PxRigidActor* px_actor0 = 0;
			if (actor0)
				px_actor0 = (PxRigidActor*)actor0->Get();

			joint = PxPrismaticJointCreate(*GetPhysics(), px_actor0, localFrame0, (PxRigidActor*)actor1->Get(), localFrame1);
			joint->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true);
		}

		PrismaticJoint(PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1) //overload for PxRigidActors
		{
			PxRigidActor* px_actor0 = 0;
			if (actor0)
				px_actor0 = (PxRigidActor*)actor0;

			joint = PxPrismaticJointCreate(*GetPhysics(), px_actor0, localFrame0, (PxRigidActor*)actor1, localFrame1);
			joint->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true);
		}

		void SetLimits(PxReal lower, PxReal upper, PxReal contactDist)
		{
			((PxPrismaticJoint*)joint)->setLimit(PxJointLinearLimitPair(PxTolerancesScale(), lower, upper, contactDist));
			((PxPrismaticJoint*)joint)->setPrismaticJointFlag(PxPrismaticJointFlag::eLIMIT_ENABLED, true);
		}

	};

	class GoalPost : public DynamicActor
	{
	public:

		//create goal post
		//center of post is 0,0,0 on ground (so adjust localPoses acordingly)
		GoalPost(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(.5f, .5f, .5f), PxReal density = 1.f)
			: DynamicActor(pose)
		{

			//create cylinders to model posts from
			Cylinder* Vcylinder = new Cylinder(15.f, .5f, 10, PxTransform(PxVec3(5.f, 5.f, 5.f)), 1.f);
			Cylinder* Hcylinder = new Cylinder(5.f, .5f, 10, PxTransform(PxVec3(5.f, 5.f, 5.f)), 1.f);

			//get meshes and create cylinders
			PxConvexMeshGeometry GeoMesh;
			Vcylinder->GetShape()->getConvexMeshGeometry(GeoMesh); //pass mesh geometry to GeoMesh
			CreateShape(GeoMesh, density); //left bar
			CreateShape(GeoMesh, density); //right bar

			Hcylinder->GetShape()->getConvexMeshGeometry(GeoMesh);
			CreateShape(GeoMesh, density);

			PxQuat QuatX = PxQuat(90.f*(PxPi / 180), PxVec3(1.0f, 0.f, 0.f)); //X rotation (upright)
			PxQuat QuatY = PxQuat(90.f*(PxPi / 180), PxVec3(.0f, 1.f, 0.f)); //Y rotation (horizontal)
			GetShape(0)->setLocalPose(PxTransform(PxVec3(2.5f, 2.f, .0f), QuatX));
			GetShape(1)->setLocalPose(PxTransform(PxVec3(-2.5f, 2.f, .0f), QuatX));
			GetShape(2)->setLocalPose(PxTransform(PxVec3(0.f, .5f, .0f), QuatY)); //-xn to account for rotation
			

		}

	};

	class RugbyPitch : public StaticActor
	{
	public:
		RugbyPitch(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(.5f, .5f, .5f), PxReal density = 1.f)
			: StaticActor(pose)
		{
			CreateShape(PxBoxGeometry(PxVec3(35.0f, .001f, .25f)), density);
			CreateShape(PxBoxGeometry(PxVec3(35.0f, .001f, .25f)), density);
			CreateShape(PxBoxGeometry(PxVec3(35.0f, .001f, .25f)), density);
			CreateShape(PxBoxGeometry(PxVec3(35.0f, .001f, .25f)), density);
			CreateShape(PxBoxGeometry(PxVec3(35.0f, .001f, .25f)), density);
			CreateShape(PxBoxGeometry(PxVec3(35.0f, .001f, .25f)), density);
			CreateShape(PxBoxGeometry(PxVec3(35.0f, .001f, .25f)), density);
			CreateShape(PxBoxGeometry(PxVec3(.25f, .001f, 55.0f)), density);
			CreateShape(PxBoxGeometry(PxVec3(.25f, .001f, 55.0f)), density);

			GetShape(0)->setLocalPose(PxTransform(PxVec3(0.f, .001f, 55.0f))); //dead-ball line
			GetShape(1)->setLocalPose(PxTransform(PxVec3(0.f, .001f, 44.0f))); //goal line
			GetShape(2)->setLocalPose(PxTransform(PxVec3(0.f, .001f, 22.0f))); //22-metre line
			GetShape(3)->setLocalPose(PxTransform(PxVec3(0.f, .001f, 0.0f)));	 //halfway line
			GetShape(4)->setLocalPose(PxTransform(PxVec3(0.f, .001f, -22.0f)));
			GetShape(5)->setLocalPose(PxTransform(PxVec3(0.f, .001f, -44.0f)));
			GetShape(6)->setLocalPose(PxTransform(PxVec3(0.f, .001f, -55.0f)));
			GetShape(7)->setLocalPose(PxTransform(PxVec3(35.f, .001f, 0.0f))); //left side
			GetShape(8)->setLocalPose(PxTransform(PxVec3(-35.f, .001f, 0.0f))); //right side
		}
	};
	
	class CatapultFrame : public DynamicActor {
	public:
		CatapultFrame(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(.5f, .5f, .5f), PxReal density = 1.f)
			: DynamicActor(pose)
		{
			//frame

			CreateShape(PxBoxGeometry(PxVec3(.25f, .25f, 4.f)), density); //left rod
			CreateShape(PxBoxGeometry(PxVec3(.25f, .25f, 4.f)), density); //right rod
			CreateShape(PxBoxGeometry(PxVec3(.25f, 1.5f, .25f)), density); //left standing rod
			CreateShape(PxBoxGeometry(PxVec3(.25f, 1.5f, .25f)), density); //right standing rod
			CreateShape(PxBoxGeometry(PxVec3(1.75f, .25f, .25f)), density);//catapult stopper
			CreateShape(PxBoxGeometry(PxVec3(1.75f, .25f, .25f)), density);//catapult holder

			GetShape(0)->setLocalPose(PxTransform(PxVec3(2.0f, .0f, .0f)));
			GetShape(1)->setLocalPose(PxTransform(PxVec3(-2.0f, .0f, .0f)));
			GetShape(2)->setLocalPose(PxTransform(PxVec3(2.0f, 1.5f, 2.0f)));
			GetShape(3)->setLocalPose(PxTransform(PxVec3(-2.0f, 1.5f, 2.0f)));
			GetShape(4)->setLocalPose(PxTransform(PxVec3(.0f, 2.75f, 2.0f)));
			GetShape(5)->setLocalPose(PxTransform(PxVec3(.0f, .0f, -3.0f)));

		}

	};

	class CatapultThrower : public DynamicActor {
	public:
		CatapultThrower(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(.5f, .5f, .5f), PxReal density = 1.f)
			: DynamicActor(pose)
		{
			
			CreateShape(PxBoxGeometry(PxVec3(.5f, .25f, 3.0f)), density);//thrower
			CreateShape(PxBoxGeometry(PxVec3(.5f, 0.4f, .1f)), density);//thrower pieces (back)
			CreateShape(PxBoxGeometry(PxVec3(.1f, 0.3f, .5f)), density);//thrower pieces
			CreateShape(PxBoxGeometry(PxVec3(.1f, 0.3f, .5f)), density);//thrower pieces
			CreateShape(PxBoxGeometry(PxVec3(.5f, 0.3f, .1f)), density);//thrower pieces

			GetShape(0)->setLocalPose(PxTransform(PxVec3(.0f, .0f, .0f))); 
			GetShape(1)->setLocalPose(PxTransform(PxVec3(.0f, .45f, -3.1f))); //back piece
			GetShape(2)->setLocalPose(PxTransform(PxVec3(-.6f, .3f, -2.5f))); //left
			GetShape(3)->setLocalPose(PxTransform(PxVec3(0.6f, .3f, -2.5f))); //right
			GetShape(4)->setLocalPose(PxTransform(PxVec3(.0f, .3f, -1.9f))); //front

		}

	};

	class CatapultWheel : public DynamicActor {
	public:
		CatapultWheel(const PxTransform& pose = PxTransform(PxIdentity), PxVec3 dimensions = PxVec3(.5f, .5f, .5f), PxReal density = 1.f)
			: DynamicActor(pose)
		{
			//create cylinder to model wheel from - length, radius, seg_number
			Cylinder* wheel = new Cylinder(.25f, .5f, 20, PxTransform(PxVec3(5.f, 5.f, 5.f)), 1.f);

			//get mesh and create wheel
			PxConvexMeshGeometry GeoMesh;
			wheel->GetShape()->getConvexMeshGeometry(GeoMesh); //pass mesh geometry to GeoMesh
			CreateShape(GeoMesh, density);

			PxQuat QuatY = PxQuat(90.f*(PxPi / 180), PxVec3(.0f, 1.f, 0.f)); //Y rotation (horizontal)
			GetShape(0)->setLocalPose(PxTransform(PxVec3(.0f, 0.f, .0f), QuatY)); //left front

		}

	};

	class Catapult {
	public:
		CatapultFrame* frame;
		CatapultWheel* frontL, *frontR, *backR, *backL;
		CatapultThrower* thrower;
		RevoluteJoint *throwerJoint;
		RevoluteJoint *jointFL, *jointFR, *jointBR, *jointBL;
		Box* ballSpawnPoint;

		float timer;

		Catapult() {
			PxVec3 pos = PxVec3(.0f, .0f, .0f);

			PxVec3 WoodColor = PxVec3(166.f / 255.f, 144.f / 255.f, 51.f / 255.f);
			PxVec3 DarkWoodColor = PxVec3(64.f / 255.f, 36.f / 255.f, 20.f / 255.f);
			CustomMaterials *mats = new CustomMaterials();

			frame = new CatapultFrame(PxTransform(pos + PxVec3(.0f, 1.0f, .0f)), PxVec3(0.5f, 0.5f, 0.5f), 40.f);
			frontL = new CatapultWheel(PxTransform(pos + PxVec3(2.5f, 0.0f, 2.5f)), PxVec3(0.5f, 0.5f, 0.5f), 40.f);
			frontR = new CatapultWheel(PxTransform(pos + PxVec3(-2.5f, 0.0f, 2.5f)), PxVec3(0.5f, 0.5f, 0.5f), 40.f);
			backR = new CatapultWheel(PxTransform(pos + PxVec3(-2.5f, 0.0f, -2.5f)), PxVec3(0.5f, 0.5f, 0.5f), 40.f);
			backL = new CatapultWheel(PxTransform(pos + PxVec3(2.5f, 0.0f, -2.5f)), PxVec3(0.5f, 0.5f, 0.5f), 40.f);
			thrower = new CatapultThrower(PxTransform(pos + PxVec3(0.0f, 1.0f, 0.0f)), PxVec3(.25f, .25f, 3.0f), 1.f);

			//to fix spawn inside throwing arm:
			ballSpawnPoint = new Box(PxTransform(pos + PxVec3(.0f, 1.5f, -3.4f)), PxVec3(0.25f, 0.25f, 0.25f), .1f);
			ballSpawnPoint->Color(DarkWoodColor);
			RevoluteJoint jointSpawn = RevoluteJoint(ballSpawnPoint, PxTransform(PxVec3(.0f, .0f, .0f), PxQuat(PxPi, PxVec3(1.f, 0.f, 0.f))), thrower, PxTransform(PxVec3(.0f, 0.f, -2.5f)));
			jointSpawn.SetLimits(0 * (PxPi / 180), 1 * (PxPi / 180));

			frame->Material(mats->woodMat);
			frontL->Material(mats->woodMat);
			frontR->Material(mats->woodMat);
			backR-> Material(mats->woodMat);
			backL->Material(mats->woodMat);
			thrower->Material(mats->woodMat);

			frame->Color(WoodColor);
			frontL->Color(DarkWoodColor);
			frontR->Color(DarkWoodColor);
			backR->Color(DarkWoodColor);
			backL->Color(DarkWoodColor);
			thrower->Color(DarkWoodColor);

			//joints connecting the wheels
			jointFL = new RevoluteJoint(frontR, PxTransform(PxVec3(.0f, .0f, .0f), PxQuat(PxPi, PxVec3(1.f, 0.f, 0.f))), frame, PxTransform(PxVec3(-2.5f, .0f, 2.0f)));
			jointFR = new RevoluteJoint(frontL, PxTransform(PxVec3(.0f, .0f, .0f), PxQuat(PxPi, PxVec3(1.f, 0.f, 0.f))), frame, PxTransform(PxVec3(2.5f, .0f, 2.0f)));
			jointBR = new RevoluteJoint(backL, PxTransform(PxVec3(.0f, .0f, .0f), PxQuat(PxPi, PxVec3(1.f, 0.f, 0.f))), frame, PxTransform(PxVec3(2.5f, .0f, -2.0f)));
			jointBL = new RevoluteJoint(backR, PxTransform(PxVec3(.0f, .0f, .0f), PxQuat(PxPi, PxVec3(1.f, 0.f, 0.f))), frame, PxTransform(PxVec3(-2.5f, .0f, -2.0f)));

			throwerJoint = new RevoluteJoint(thrower, PxTransform(PxVec3(0.f, 0.f, 3.0f), PxQuat(PxPi * 2, PxVec3(1.f, 0.f, 0.f))), frame, PxTransform(PxVec3(.0f, .0f, 2.0f)));
			throwerJoint->SetLimits(-80 * (PxPi / 180), -6 * (PxPi / 180));

		}

		void AddToScene(Scene* scene)
		{
			scene->Add(frame);
			scene->Add(frontL);
			scene->Add(frontR);
			scene->Add(backR);
			scene->Add(backL);
			scene->Add(thrower);
			scene->Add(ballSpawnPoint);
		}
	};

	class Portcullis {

		Box *holder;
		PrismaticJoint *prismaticJoint;

	public:
		Box *gate;

		Portcullis(PxVec3 pos = PxVec3(.0f, .0f, .0f), float scale = 1.f) {

			PxVec3 metalic = PxVec3(51.f / 255.f, 65.f / 255.f, 71.f / 255.f);

			holder = new Box(PxTransform(pos + PxVec3(.0f, 0.0f, .0f)), PxVec3(5.0f, .25f, .25f)*scale, 1.f);
			holder->SetKinematic(true);
			gate = new Box(PxTransform(pos + PxVec3(.0f, 0.0f, .0f)), PxVec3(5.0f, 3.0f, .25f)*scale, 000.1f);

			gate->Color(metalic);

			prismaticJoint = new PrismaticJoint(holder, PxTransform(PxVec3(.0f, .0f, .25f)), gate, PxTransform(PxVec3(.0f, 5.5f, -.25f)));
			prismaticJoint->SetLimits(-20.0f, 20.0f, 0.01f);

		}

		void AddToScene(Scene* scene) {
			scene->Add(gate);
			scene->Add(holder);
		}
	};


}
