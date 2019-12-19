
//comment from Snippet
/*It shows the setup of MBPand its regions.In this example 4 regions are setup
and set for the MBP.Created stacks are then simulated in multiple regions.
Note that current regions setup is not optimal, some objects get out of regions bounds.
In this case a warning is reported.It is possible to add PxBroadPhaseCallback
to scene to handle such cases.*/

//https://gameworksdocs.nvidia.com/PhysX/4.1/documentation/physxguide/Manual/RigidBodyCollision.html#broad-phase-callback

#include <ctype.h>
#include <vector>

#include "PxPhysicsAPI.h"

#include "SnippetUtils.h"
#include "SnippetPrint.h"
#include "SnippetPVD.h"


using namespace physx;

PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;

PxFoundation* gFoundation = NULL;
PxPhysics* gPhysics = NULL;

PxDefaultCpuDispatcher* gDispatcher = NULL;
PxScene* gScene = NULL;

PxMaterial* gMaterial = NULL;

PxPvd* gPvd = NULL;

PxReal stackZ = 10.0f;

PxU32 gRegionHandles[4];

//슈팅볼 생성.
PxRigidDynamic* CreateDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity = PxVec3(0))
{
	PxRigidDynamic* ball = nullptr;
	PxShape* shape = gPhysics->createShape(geometry, *gMaterial, true);
	{
		if (!shape)
		{
			return nullptr;
		}

		ball = gPhysics->createRigidDynamic(t);

		ball->attachShape(*shape);
		PxRigidBodyExt::updateMassAndInertia(*ball, 10.0f);
	}
	shape->release();

	ball->setAngularDamping(0.5f);
	ball->setLinearVelocity(velocity);
	ball->setName("Ball");
	gScene->addActor(*ball);
	return ball;
}

void CreateStack(const PxTransform& t, PxU32 size, PxReal halfExtent)
{
	PxShape* shape = gPhysics->createShape(PxBoxGeometry(halfExtent, halfExtent, halfExtent), *gMaterial);
	for (PxU32 i = 0; i < size; i++)
	{
		for (PxU32 j = 0; j < size - i; j++)
		{
			PxTransform localTm(PxVec3(PxReal(j * 2) - PxReal(size - i), PxReal(i * 2 + 1), 0) * halfExtent);
			PxRigidDynamic* body = gPhysics->createRigidDynamic(t.transform(localTm));
			body->attachShape(*shape);
			PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
			gScene->addActor(*body);
		}
	}
	shape->release();
}

class SnippetMBPBroadPhaseCallback : public physx::PxBroadPhaseCallback
{
public:
	virtual ~SnippetMBPBroadPhaseCallback() = default;

	virtual	void onObjectOutOfBounds(PxShape& shape, PxActor& actor) override
	{
		if (actor.getName() != "Ball")
		{
			bool isNoneOverlapActor = true;
			for (auto& it : m_OutActors)
			{
				if (it == &actor)
				{
					isNoneOverlapActor = false;
					break;
				}
			}

			if (isNoneOverlapActor)
			{
				m_OutActors.push_back(&actor);
			}
		}
	}

	virtual	void onObjectOutOfBounds(PxAggregate& aggregate) override
	{
		
	}

	void PurgeOutActors()
	{
		for (auto& it : m_OutActors)
		{
			it->release();
		}

		m_OutActors.clear();
	}

private:
	std::vector<PxActor*> m_OutActors;

} gBroadPhaseCallback;


void InitPhysics(bool interactive)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* pvdTransport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*pvdTransport, PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());

	sceneDesc.gravity = PxVec3(0, -9.81f, 0);

	PxU32 numCores = SnippetUtils::getNbPhysicalCores();
	gDispatcher = PxDefaultCpuDispatcherCreate(numCores == 0 ? 0 : numCores - 1);
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;

	//https://gameworksdocs.nvidia.com/PhysX/4.1/documentation/physxguide/Manual/RigidBodyCollision.html#broad-phase-algorithms
	//검사 알고리즘 선택.
	sceneDesc.broadPhaseType = PxBroadPhaseType::eMBP;
	sceneDesc.broadPhaseCallback = &gBroadPhaseCallback;

	gScene = gPhysics->createScene(sceneDesc);

	// 경계지역 생성.
	PxBroadPhaseRegion regions[4] =
	{
		{	PxBounds3(PxVec3(-100, -100, -100),  PxVec3(0, 100,   0)), reinterpret_cast<void*>(1) },
		{	PxBounds3(PxVec3(-100, -100,    0),  PxVec3(0, 100, 100)), reinterpret_cast<void*>(2) },
		{	PxBounds3(PxVec3(0, -100, -100),  PxVec3(100, 100,   0)), reinterpret_cast<void*>(3) },
		{	PxBounds3(PxVec3(0, -100,    0),  PxVec3(100, 100, 100)), reinterpret_cast<void*>(4) }
	};

	for (auto& it : regions)
	{
		gScene->addBroadPhaseRegion(it);
	}

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if (pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}

	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	PxRigidStatic* ground = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);

	gScene->addActor(*ground);

	for (PxU32 i = 0; i < 5; i++)
	{
		CreateStack(PxTransform(PxVec3(0, 0, stackZ -= 10.0f)), 10, 2.0f);
	}

	if (!interactive)
	{
		CreateDynamic(PxTransform(PxVec3(0, 40, 100)), PxSphereGeometry(10), PxVec3(0, -50, -100));
	}
}

void StepPhysics(bool)
{
	gScene->simulate(1 / 60.0f);
	gScene->fetchResults(true);

	// 바운드 밖으로 나간 액터들 전부 릴리즈.
	gBroadPhaseCallback.PurgeOutActors();
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

	printf("SnippetMBP done.\n");
}

void KeyPress(unsigned char key, const PxTransform& camera)
{
	switch (toupper(key))
	{
	case 'B':	CreateStack(PxTransform(PxVec3(0, 0, stackZ -= 10.0f)), 10, 2.0f);						break;
	case ' ':	CreateDynamic(camera, PxSphereGeometry(3.0f), camera.rotate(PxVec3(0, 0, -1)) * 200);	break;
	}
}

int SnippetMain(int, const char* const*)
{
#ifdef RENDER_SNIPPET
	extern void RenderLoop();
	RenderLoop();
#else
	static const PxU32 frameCount = 100;
	initPhysics(false);
	for (PxU32 i = 0; i < frameCount; i++)
		stepPhysics(false);
	cleanupPhysics(false);
#endif

	return 0;
}
