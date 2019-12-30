#include <ctype.h>

#include "PxPhysicsAPI.h"

#include "SnippetPrint.h"
#include "SnippetPVD.h"
#include "SnippetUtils.h"

using namespace physx;

PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;

PxFoundation* gFoundation = NULL;
PxPhysics* gPhysics = NULL;
PxCooking* gCooking = NULL;

PxDefaultCpuDispatcher* gDispatcher = NULL;
PxScene* gScene = NULL;

PxMaterial* gMaterial = NULL;

PxPvd* gPvd = NULL;

PxTriangleMesh* gMesh = NULL;
PxRigidStatic* gActor = NULL;

PxReal stackZ = 10.0f;

static const PxU32 gGridSize = 8;
static const PxReal gGridStep = 512.0f / PxReal(gGridSize - 1);
static float gTime = 0.0f;

struct Triangle
{
	PxU32 ind0, ind1, ind2;
};

static PxRigidDynamic* CreateDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity = PxVec3(0), PxReal density = 1.0f)
{
	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, density);
	dynamic->setLinearVelocity(velocity);
	gScene->addActor(*dynamic);
	return dynamic;
}

static void CreateStack(const PxTransform& t, PxU32 size, PxReal halfExtent)
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

static void UpdateVertices(PxVec3* verts, float amplitude = 0.0f)
{
	const PxU32 gridSize = gGridSize;
	const PxReal gridStep = gGridStep;

	for (PxU32 a = 0; a < gridSize; a++)
	{
		const float coeffA = float(a) / float(gridSize);
		for (PxU32 b = 0; b < gridSize; b++)
		{
			const float coeffB = float(b) / float(gridSize);

			const float y = 20.0f + sinf(coeffA * PxTwoPi) * cosf(coeffB * PxTwoPi) * amplitude;

			verts[a * gridSize + b] = PxVec3(-400.0f + b * gridStep, y, -400.0f + a * gridStep);
		}
	}
}

static PxTriangleMesh* CreateMeshGround()
{
	PxTriangleMesh* result = nullptr;

	const PxU32 gridSize = gGridSize;

	PxVec3 verts[gridSize * gridSize];

	const PxU32 nbTriangles = 2 * (gridSize - 1) * (gridSize - 1);

	Triangle indices[nbTriangles];

	UpdateVertices(verts);

	for (PxU32 a = 0; a < (gridSize - 1); ++a)
	{
		for (PxU32 b = 0; b < (gridSize - 1); ++b)
		{
			Triangle& tri0 = indices[(a * (gridSize - 1) + b) * 2];
			Triangle& tri1 = indices[((a * (gridSize - 1) + b) * 2) + 1];

			tri0.ind0 = a * gridSize + b + 1;
			tri0.ind1 = a * gridSize + b;
			tri0.ind2 = (a + 1) * gridSize + b + 1;

			tri1.ind0 = (a + 1) * gridSize + b + 1;
			tri1.ind1 = a * gridSize + b;
			tri1.ind2 = (a + 1) * gridSize + b;
		}
	}

	PxTriangleMeshDesc triangleMeshDesc;
	triangleMeshDesc.points.data = verts;
	triangleMeshDesc.points.stride = sizeof(PxVec3);
	triangleMeshDesc.points.count = gridSize * gridSize;
	triangleMeshDesc.triangles.count = nbTriangles;
	triangleMeshDesc.triangles.data = indices;
	triangleMeshDesc.triangles.stride = sizeof(Triangle);

	result = gCooking->createTriangleMesh(triangleMeshDesc, gPhysics->getPhysicsInsertionCallback());
	return result;
}

void InitPhysics(bool)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);

	gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);

	PxCookingParams cookingParams(gPhysics->getTolerancesScale());

	// Deformable meshs는 PxMeshMidPhase::eBVH33 에서만 지원된다.
	cookingParams.midphaseDesc.setToDefault(PxMeshMidPhase::eBVH33);
	cookingParams.meshPreprocessParams = PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH;

	gCooking = PxCreateCooking(PX_PHYSICS_VERSION, *gFoundation, cookingParams);


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

	PxTriangleMesh* mesh = CreateMeshGround();
	gMesh = mesh;

	PxTriangleMeshGeometry geom(mesh);

	PxRigidStatic* groundMesh = gPhysics->createRigidStatic(PxTransform(PxVec3(0, 2, 0)));
	gActor = groundMesh;
	PxShape* shape = gPhysics->createShape(geom, *gMaterial);

	{
		shape->setContactOffset(0.02f);
		shape->setRestOffset(-0.5f);
	}

	groundMesh->attachShape(*shape);
	gScene->addActor(*groundMesh);

	CreateStack(PxTransform(PxVec3(0, 22, 0)), 10, 2.0f);
}

void StepPhysics(bool)
{
	{
		PxVec3* verts = gMesh->getVerticesForModification();
		gTime += 0.01f;
		UpdateVertices(verts, sinf(gTime) * 20.0f);
		PxBounds3 newBounds = gMesh->refitBVH();
		PX_UNUSED(newBounds);

		gScene->resetFiltering(*gActor);
	}

	gScene->simulate(1.0f / 60.0f);
	gScene->fetchResults(true);
}

void CleanupPhysics(bool)
{
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
	PX_RELEASE(gCooking);
	if (gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		gPvd->release();	gPvd = NULL;
		PX_RELEASE(transport);
	}
	PX_RELEASE(gFoundation);

	printf("SnippetDeformableMesh done.\n");
}

void KeyPress(unsigned char key, const PxTransform& camera)
{
	switch (toupper(key))
	{
	case ' ':	CreateDynamic(camera, PxSphereGeometry(3.0f), camera.rotate(PxVec3(0, 0, -1)) * 200, 3.0f);	break;
	}
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
