#pragma once

#include "BasicActors.h"
#include "CustomActors.h"
#include "SZ_PxBox.h"
#include <iostream>
#include <iomanip>

#include "VisualDebugger.h"


namespace PhysicsEngine
{
	using namespace std;

	static const string ballName = "ball";
	static const string redGoalName = "redGoal";
	static const string blueGoalName = "blueGoal";

	//a list of colours: Circus Palette
	static const PxVec3 color_palette[] = {
		PxVec3(46.f / 255.f,9.f / 255.f,39.f / 255.f),
		PxVec3(185.f / 255.f, 0.f / 255.f, 0.f / 255.f),	//red
		PxVec3(0.f / 255.f, 0.f / 255.f, 185.f / 255.f), //blue
		PxVec3(255.f / 255.f,140.f / 255.f,54.f / 255.f),
		PxVec3(4.f / 255.f,117.f / 255.f,111.f / 255.f),
		PxVec3(166.f / 255.f,144.f / 255.f,51.f / 255.f), //wood
		PxVec3(64.f / 255.f, 36.f / 255.f, 20.f / 255.f) //dark wood
	};

	struct FilterGroup
	{
		enum Enum
		{
			ACTOR0		= (1 << 0),
			ACTOR1		= (1 << 1),
			ACTOR2		= (1 << 2)
			//add more if you need
		};
	};

	///A customised collision class, implemneting various callbacks
	class MySimulationEventCallback : public PxSimulationEventCallback
	{
	public:
		//an example variable that will be checked in the main simulation loop
		bool redGoalTrigger;
		bool blueGoalTrigger;

		MySimulationEventCallback() : redGoalTrigger(false), blueGoalTrigger(false){}

		///Method called when the contact with the trigger object is detected.
		virtual void onTrigger(PxTriggerPair* pairs, PxU32 count) 
		{
			//you can read the trigger information here
			for (PxU32 i = 0; i < count; i++)
			{
				//filter out contact with the planes
				if (pairs[i].otherShape->getGeometryType() != PxGeometryType::ePLANE)
				{
					//check if eNOTIFY_TOUCH_FOUND trigger
					if (pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_FOUND)
					{
						//cerr << "onTrigger::eNOTIFY_TOUCH_FOUND" << endl;

						if (pairs[i].otherActor->getName() == ballName) { //ball collisions
							if (pairs[i].triggerActor->getName() == redGoalName)
								redGoalTrigger = true;
							if (pairs[i].triggerActor->getName() == blueGoalName)
								blueGoalTrigger = true;
						}
						
						//cerr << pairs[i].triggerActor->getName() << endl;
						//cerr << pairs[i].otherActor->getName() << endl;
					}
					//check if eNOTIFY_TOUCH_LOST trigger
					if (pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_LOST)
					{
						//cerr << "onTrigger::eNOTIFY_TOUCH_LOST" << endl;
						//trigger = false;
					}
				}
			}
		}

		///Method called when the contact by the filter shader is detected.
		virtual void onContact(const PxContactPairHeader &pairHeader, const PxContactPair *pairs, PxU32 nbPairs) 
		{
			cerr << "Contact found between " << pairHeader.actors[0]->getName() << " " << pairHeader.actors[1]->getName() << endl;

			//check all pairs
			for (PxU32 i = 0; i < nbPairs; i++)
			{
				//check eNOTIFY_TOUCH_FOUND
				if (pairs[i].events & PxPairFlag::eNOTIFY_TOUCH_FOUND)
				{
					cerr << "onContact::eNOTIFY_TOUCH_FOUND" << endl;
				}
				//check eNOTIFY_TOUCH_LOST
				if (pairs[i].events & PxPairFlag::eNOTIFY_TOUCH_LOST)
				{
					cerr << "onContact::eNOTIFY_TOUCH_LOST" << endl;
				}
			}
		}

		virtual void onConstraintBreak(PxConstraintInfo *constraints, PxU32 count) {}
		virtual void onWake(PxActor **actors, PxU32 count) {}
		virtual void onSleep(PxActor **actors, PxU32 count) {}
	};

	//A simple filter shader based on PxDefaultSimulationFilterShader - without group filtering
	static PxFilterFlags CustomFilterShader( PxFilterObjectAttributes attributes0,	PxFilterData filterData0,
		PxFilterObjectAttributes attributes1,	PxFilterData filterData1,
		PxPairFlags& pairFlags,	const void* constantBlock,	PxU32 constantBlockSize)
	{
		// let triggers through
		if(PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1))
		{
			pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
			return PxFilterFlags();
		}

		pairFlags = PxPairFlag::eCONTACT_DEFAULT;
		//enable continous collision detection
//		pairFlags |= PxPairFlag::eCCD_LINEAR;
		
		
		//customise collision filtering here
		//e.g.

		// trigger the contact callback for pairs (A,B) where 
		// the filtermask of A contains the ID of B and vice versa.
		if((filterData0.word0 & filterData1.word1) && (filterData1.word0 & filterData0.word1))
		{
			//trigger onContact callback for this pair of objects
			pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND;
			pairFlags |= PxPairFlag::eNOTIFY_TOUCH_LOST;
//			pairFlags |= PxPairFlag::eNOTIFY_CONTACT_POINTS;
		}

		return PxFilterFlags();
	};

	///Custom scene class
	class MyScene : public Scene
	{
		CustomMaterials* customMaterials = new CustomMaterials();
		Plane* plane;
		Box* redGoal, *blueGoal;
		MySimulationEventCallback* my_callback;
		RugbyPitch* pitch;
		GoalPost* redPost, *bluePost;
		CatapultFrame* frame;
		Catapult* catapult;
		PxRigidDynamic* gate;
		PxRigidDynamic* gate2;

		PxReal deltaTime;
		float timer;
		int gateDir;
		
	public:

		//specify your custom filter shader here
		//PxDefaultSimulationFilterShader by default
		MyScene() : Scene() {};

		///A custom scene class
		void SetVisualisation() //debug mode visualisation types
		{
			px_scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);     //to see collsion shapes
			px_scene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LOCAL_FRAMES, 1.0f);	// to visualise joint local axes;
			px_scene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LIMITS, 1.0f);			//to visualise the limits of joints.
		}

		//collision stuff moved from CustomInit():
		//set collision filter flags
			// box->SetupFiltering(FilterGroup::ACTOR0, FilterGroup::ACTOR1);
			//use | operator to combine more actors e.g.
			// box->SetupFiltering(FilterGroup::ACTOR0, FilterGroup::ACTOR1 | FilterGroup::ACTOR2);
			//don't forget to set your flags for the matching actor as well, e.g.:
			// box2->SetupFiltering(FilterGroup::ACTOR1, FilterGroup::ACTOR0);

		//Custom scene initialisation
		virtual void CustomInit() 
		{

			SetVisualisation();			

			GetMaterial()->setDynamicFriction(.2f);

			///Initialise and set the customised event callback
			my_callback = new MySimulationEventCallback();
			px_scene->setSimulationEventCallback(my_callback);

			plane = new Plane();
			plane->Color(PxVec3(140.f/255.f, 193.f/255.f, 63.f/255.f));
			plane->Material(customMaterials->grassMat);
			Add(plane);
			
			Add(new RugbyPitch());
			redPost = new GoalPost(PxTransform(PxVec3(0.f, 5.5f, 55.0f)));
			redPost->Material(customMaterials->steelMat);
			bluePost = new GoalPost(PxTransform(PxVec3(0.f, 5.5f, -55.0f)));
			bluePost->Material(customMaterials->steelMat);
			Add(redPost);
			Add(bluePost);

			//create catapult
			catapult = new Catapult();
			catapult->AddToScene(this);
			
			PxRigidDynamic* px_actor = (PxRigidDynamic*)catapult->thrower->Get();
			PxVec3 posOfThrower = px_actor->getGlobalPose().p;
	
			//spawn rugby ball
			spawnBall();

			//create invisible goal collision detection objects
			//make sure they are not touching GoalPosts, otherwise triggers collision
			PxTransform redGoalPos = ((PxRigidBody*)redPost->Get())->getGlobalPose();
			redGoalPos.p += PxVec3(.0f, 5.0f, .0f); //add positon we want the goal section to be
			PxTransform blueGoalPos = ((PxRigidBody*)bluePost->Get())->getGlobalPose();
			blueGoalPos.p += PxVec3(.0f, 5.0f, .0f);
			
			Box* redGoal = new Box(redGoalPos, PxVec3(1.90f, 3.90f, .5f), 1.f);
			redGoal->SetKinematic(true);
			redGoal->SetTrigger(true);
			redGoal->Name(redGoalName);
			Add(redGoal);

			Box* blueGoal = new Box(blueGoalPos, PxVec3(1.90f, 3.90f, .5f), 1.f);
			blueGoal->SetKinematic(true);
			blueGoal->SetTrigger(true);
			blueGoal->Name(blueGoalName);
			Add(blueGoal);

			//sliding door obstacles
			Portcullis *portcullis = new Portcullis(PxVec3(.0f, 15.00f, -40.0f), 2.f);
			portcullis->AddToScene(this);
			Portcullis *portcullis2 = new Portcullis(PxVec3(.0f, 15.00f, 40.0f), 2.f);
			portcullis2->AddToScene(this);
			timer = deltaTime + 3.f; //reset timer
			gateDir = 1; //set gate to move right first

			gate = (PxRigidDynamic*)portcullis->gate->Get();
			gate->addForce(PxVec3(1, 0, 0)*10000.f); // start sliding door moving
			gate2 = (PxRigidDynamic*)portcullis2->gate->Get();
			gate2->addForce(PxVec3(1, 0, 0)*10000.f); // start sliding door moving


			PxQuat QuatZ = PxQuat(45.f*(PxPi / 180), PxVec3(.0f, 0.f, 1.f));
			//red team flags
			Cloth* redCloth = new Cloth(PxTransform(PxVec3(20.f, 9.f, 55.f), QuatZ), PxVec2(4.f, 4.f), 7, 7);
			Cloth* redCloth2 = new Cloth(PxTransform(PxVec3(-20.f, 9.f, 55.f), QuatZ), PxVec2(4.f, 4.f), 7, 7);
			redCloth->Color(color_palette[1]);
			redCloth2->Color(color_palette[1]);
			Add(redCloth);
			Add(redCloth2);

			//blue team flags
			Cloth* blueCloth = new Cloth(PxTransform(PxVec3(20.f, 9.f, -55.f), QuatZ), PxVec2(4.f, 4.f), 7, 7);
			Cloth* blueCloth2 = new Cloth(PxTransform(PxVec3(-20.f, 9.f, -55.f), QuatZ), PxVec2(4.f, 4.f), 7, 7);
			blueCloth->Color(color_palette[2]);
			blueCloth2->Color(color_palette[2]);
			Add(blueCloth);
			Add(blueCloth2);

		}

		//Custom update function
		virtual void CustomUpdate() 
		{
			deltaTime = deltaTime + 1.0f / 60.0f;

			if (deltaTime > catapult->timer && catapult->timer != NULL) {
				catapult->throwerJoint->DriveVelocity(5.0f);
				catapult->timer = NULL;
			}

			if (deltaTime > timer) {
				timer = deltaTime + 6.f; //reset timer 
				gateDir *= -1; //swtich direction
				gate->addForce(PxVec3(gateDir, 0, 0)*10000.f); //move gate () 
				gate2->addForce(PxVec3(gateDir, 0, 0)*10000.f); //move gate () 
			}

			if (my_callback->redGoalTrigger == true) {
				redTeamScore += 3;
				cerr << "red goal scored!" << endl;
				//VisualDebugger::UpdateHUD(VisualDebugger::HUDState::BASE);
				my_callback->redGoalTrigger = false;
			}

			if (my_callback->blueGoalTrigger == true) {
				blueTeamScore += 3;
				my_callback->blueGoalTrigger = false;
				cerr << "blue goal scored!" << endl;
				//VisualDebugger::UpdateHUD(VisualDebugger::BASE);
				my_callback->blueGoalTrigger = false;
			}
				
		}

		void catapultTriggerHandler()
		{
			cerr << "fire catapult" << endl;
			catapult->throwerJoint->DriveVelocity(-5.0f);
			catapult->timer = deltaTime + 2.0f; 
		}

		void spawnBall()
		{
			PxRigidDynamic* px_actor = (PxRigidDynamic*)catapult->ballSpawnPoint->Get();
			PxTransform poseOfspawn = px_actor->getGlobalPose();

			Ellipsoid *newBall = new Ellipsoid(.65f, 12, 8, PxTransform(poseOfspawn.p + PxVec3(.0f, 1.3f, 0.f), poseOfspawn.q), .1f);

			//Sphere *newBall = new Sphere(PxTransform(poseOfspawn.p + PxVec3(.0f, 1.3f, 0.f), poseOfspawn.q), .4f, .08f); //rugby ball - density 0.08kg/m3
			newBall->Material(customMaterials->ballMat);
			newBall->Color(color_palette[1]);
			newBall->Name(ballName);
			Add(newBall);
		}

		void catapultForward() {
			catapult->jointFL->DriveVelocity(-10.f);
			catapult->jointFR->DriveVelocity(-10.f);
		}
		void catapultBack() {
			catapult->jointFL->DriveVelocity(10.f);
			catapult->jointFR->DriveVelocity(10.f);
		}
		void catapultLeftTurn() {
			catapult->jointFL->DriveVelocity(-25.f);
		}
		void catapultRightTurn() {
			catapult->jointFR->DriveVelocity(-25.f);
		}

		void catapultStop() {
			//turn off all drive velocities		
			catapult->jointFL->DriveVelocity(0.f);
			catapult->jointFR->DriveVelocity(0.f);	
		}

	};

}
