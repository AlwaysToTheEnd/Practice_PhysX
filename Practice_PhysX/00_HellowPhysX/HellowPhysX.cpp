
#include <Windows.h>
#include <ctype.h>

#include "PxPhysicsAPI.h"

#include "SnippetPrint.h"
#include "SnippetPVD.h"
#include "SnippetUtils.h"

using namespace physx;

PxDefaultAllocator gAllocator;

PxDefaultErrorCallback gErrorCallback;

PxFoundation* gFoundation = nullptr;
PxPhysics* gPhysics = nullptr;

PxDefaultCpuDispatcher* gDispatcher = nullptr;
PxScene* gScene = nullptr;

PxMaterial* gMaterial = nullptr;

PxPvd* gPvd = nullptr;

PxReal stackZ = 10.0f;

PxRigidDynamic* CreateDynamic(const PxTransform& t, const PxGeometry& geometry,
	const PxVec3& velocity = PxVec3(0));
void CreateStack(const PxTransform& t, PxU32 size, PxReal halfExtent);

void InitPhysics(bool interactive)
{
	// 가장 기초가 되는 Foundation 생성, 싱글톤 클래스임에 주의하자.
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	// Physics Visual Debugger에 연결하기 포트 생성및 연결 작업. 
	// 사용하지 않는다면 하지 않아도 무방하다.
	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

	// 모든 Physics 리소스 생성을 위한 메인 클래스 생성, 싱글톤 클래스임에 주의하자.
	// DirectX Device와 같은 개념으로 이해하면 좋다.
	// PxTolerancesScale 모든 물리처리에 있어서 기준의 되는 단위 설정.
	// length(1) 
	// mass(1000) 
	// speed(10) 
	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);

	// CPU작업을 어떤식으로 할지 정한다. 씬에 적용
	// 쓰레드의 수를 정한다.
	SYSTEM_INFO info = {};
	GetSystemInfo(&info);

	gDispatcher = PxDefaultCpuDispatcherCreate(info.dwNumberOfProcessors);

	// 오브젝트들이 시뮬레이션 될 공간을 생성한다.
	// 씬 안에 시뮬레이션 될 액터 등 이 담기고 씬을 기준으로 시뮬레이션 한다.
	// PxSceneDesc는 씬의 속성 정보를 담은 구조체이다.
	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;

	// 씬에서의 시뮬레이션,위상 이벤트등을 콜백 인터페이스로 제어한다

	// 콜백규칙
	// 콜백은 메인스레드 또는 시뮬레이션 스레드에서 동시에 실행될 수 있으므로
	// SDK의 상태를 수정하면 안되며 특히 객체를 생성하거나 파괴해서는 안된다
	// 상태 수정이 필요한 경우 변경 내용을 버퍼에 저장 후 시뮬레이션 단계 이후 수행할 것

	//sceneDesc.simulationEventCallback
	gScene = gPhysics->createScene(sceneDesc);
	
	//pvd 클라이언트 세팅
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

	for (PxU32 i = 0; i < 5; i++)
	{
		CreateStack(PxTransform(PxVec3(0, 0, stackZ -= 10.0f)), 10, 2.0f);
	}

	if (!interactive)
	{
		CreateDynamic(PxTransform(PxVec3(0, 40, 100)), PxSphereGeometry(10), PxVec3(0, -50, -100));
	}
}


PxRigidDynamic* CreateDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity)
{
	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 10.0f);
	dynamic->setAngularDamping(0.5f);
	dynamic->setLinearVelocity(velocity);

	gScene->addActor(*dynamic);

	return dynamic;
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
		gPvd->release();
		gPvd = nullptr;
		PX_RELEASE(transport);
	}
	PX_RELEASE(gFoundation);
}

void KeyPress(unsigned char key, const PxTransform& camera)
{
	switch (toupper(key))
	{
	case 'B':	CreateStack(PxTransform(PxVec3(0, 0, stackZ -= 10.0f)), 10, 2.0f);						break;
	case ' ':	CreateDynamic(camera, PxSphereGeometry(3.0f), camera.rotate(PxVec3(0, 0, -1)) * 200);	break;
	}
}

int snippetMain(int, const char* const*)
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