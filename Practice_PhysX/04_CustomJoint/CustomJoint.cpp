#include <ctype.h>

#include "PxPhysicsAPI.h"

#include "SnippetPrint.h"
#include "SnippetPVD.h"
#include "SnippetUtils.h"

#include "PulleyJoint.h"

using namespace physx;

PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;

PxFoundation* gFoundation = NULL;
PxPhysics* gPhysics = NULL;

PxDefaultCpuDispatcher* gDispatcher = NULL;
PxScene* gScene = NULL;

PxMaterial* gMaterial = NULL;
PxPvd* gPvd = NULL;

void InitPhysics(bool)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;
	gScene = gPhysics->createScene(sceneDesc);

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if (pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}

	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
	gScene->addActor(*groundPlane);


	PxBoxGeometry boxGeom(1.0f, 1.0f, 1.0f);
	PxRigidDynamic* box0 = PxCreateDynamic(*gPhysics, PxTransform(PxVec3(5, 5, 0)), boxGeom, *gMaterial, 1.0f);
	PxRigidDynamic* box1 = PxCreateDynamic(*gPhysics, PxTransform(PxVec3(0, 5, 0)), boxGeom, *gMaterial, 2.0f);

	PulleyJoint* joint = new PulleyJoint(*gPhysics, *box0, PxTransform(PxVec3(0.0f, 1.0f, 0.0f)), PxVec3(10.0f, 20.0f, 0.0f),
		*box1, PxTransform(PxVec3(0.0f, 1.0f, 0.0f)), PxVec3(0.0f, 20.0f, 0.0f));

	joint->SetDistance(10.0f);

	gScene->addActor(*box0);
	gScene->addActor(*box1);
}

void StepPhysics(bool)
{
	gScene->simulate(1.0f / 60.0f);
	gScene->fetchResults(true);
}

void CleanupPhysics(bool)
{
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
	if (gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		gPvd->release();	gPvd = NULL;
		PX_RELEASE(transport);
	}
	PX_RELEASE(gFoundation);

	printf("SnippetCustomJoint done.\n");
}

void KeyPress(unsigned char, const PxTransform&)
{
}

int SnippetMain(int, const char* const*)
{
#ifdef RENDER_SNIPPET
	extern void RenderLoop();
	RenderLoop();
#else
	static const PxU32 frameCount = 100;
	InitPhysics(false);
	for (PxU32 i = 0; i < frameCount; i++)
		StepPhysics(false);
	CleanupPhysics(false);
#endif

	return 0;
}
