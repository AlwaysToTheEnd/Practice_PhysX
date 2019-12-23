//comment from Snippet
/* It defines a filter shader function that requests touch reports for 
all pairs, and a contact callback function that saves the contact points.  
It configures the scene to use this filter and callback, and prints the 
number of contact reports each frame. If rendering, it renders each 
contact as a line whose length and direction are defined by the contact 
impulse.*/

#include <vector>

#include <PxPhysicsAPI.h>

#include "SnippetPrint.h"
#include "SnippetPVD.h"
#include "SnippetUtils.h"
#include "task/PxTask.h"
#include <atomic>

#define PARALLEL_CALLBACKS 1

using namespace physx;

PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;

PxFoundation*			gFoundation=nullptr;
PxPhysics*				gPhysics=nullptr;

PxDefaultCpuDispatcher* gDispatcher = nullptr;
PxScene*				gScene=nullptr;
PxMaterial*				gMaterial=nullptr;
PxPvd*					gPvd=nullptr;

const PxI32 maxCount = 10000;

std::atomic_int32_t gSharedIndex = 0;

std::vector<PxVec3> gContactPositions;
std::vector<PxVec3> gContactImpulses;
std::vector<PxVec3> gContactVertices;


class CallbackFinishTask :public PxLightCpuTask
{
	SnippetUtils::Sync* m_Sync;

public:
	CallbackFinishTask() { m_Sync = SnippetUtils::syncCreate(); }

	virtual void release() override
	{
		PxLightCpuTask::release();
		SnippetUtils::syncSet(m_Sync);
	}

	void Reset() { SnippetUtils::syncReset(m_Sync); }
	void Wait() { SnippetUtils::syncWait(m_Sync); }
	virtual void run() override {}
	virtual const char* getName()const { return "CallbackFinishTask"; }

} gCallbackFinishTask;

//충돌 처리에 사용되는 필터 세이더 작성. 기존 PxDefaultSimulationFilterShader 대신 사용된다.
PxFilterFlags ContactReportFilterShader(PxFilterObjectAttributes, PxFilterData,
	PxFilterObjectAttributes, PxFilterData,
	PxPairFlags& pairFlags, const void*, PxU32)
{
	//https://gameworksdocs.nvidia.com/PhysX/4.1/documentation/physxapi/files/structPxPairFlag.html#a60e71a2948b030140f840766a3f7ac2f
	
	pairFlags = PxPairFlag::eSOLVE_CONTACT 
		| PxPairFlag::eDETECT_DISCRETE_CONTACT
		| PxPairFlag::eNOTIFY_TOUCH_FOUND
		| PxPairFlag::eNOTIFY_TOUCH_PERSISTS
		| PxPairFlag::eNOTIFY_CONTACT_POINTS;

	return PxFilterFlag::eDEFAULT;
}

//시뮬레이션 이벤트에서 사용될 콜백함수들 작성.
//아래에서는 충돌됐을때의 상태만 정의한다.
class ContactReportCallback :public PxSimulationEventCallback
{
	virtual void onConstraintBreak(PxConstraintInfo*, PxU32) override {}
	virtual void onWake(PxActor**, PxU32) override {}
	virtual void onSleep(PxActor**, PxU32) override {}
	virtual void onTrigger(PxTriggerPair*, PxU32) override {}
	virtual void onAdvance(const PxRigidBody* const*, const PxTransform*, const PxU32) override {}
	virtual void onContact(const PxContactPairHeader& pairHeader, const PxContactPair* pairs, PxU32 nbPairs) override
	{
		//접점 생성시 최대 64개까지 생성가능.
		PxContactPairPoint contactPoints[64];

		for (PxU32 i = 0; i < nbPairs; i++)
		{
			PxU32 contactCount = pairs[i].contactCount;
			if (contactCount)
			{
				pairs[i].extractContacts(&contactPoints[0], contactCount);

				PxI32 startIdx = gSharedIndex.fetch_add(contactCount);
				for (PxU32 j = 0; j < contactCount; j++)
				{
					gContactPositions[startIdx + j] = contactPoints[j].position;
					gContactImpulses[startIdx + j] = contactPoints[j].impulse;
					gContactVertices[2 * (startIdx + j)] = contactPoints[j].position;
					gContactVertices[2 * (startIdx + j) + 1] = contactPoints[j].position + contactPoints[j].impulse * 0.1f;
				}
			}
		}
	}
} gContactReportCallback;

void CreateStack(const PxTransform& t, PxU32 size, PxReal harfExtent)
{
	PxShape* shape = gPhysics->createShape(
		PxBoxGeometry(harfExtent, harfExtent, harfExtent), *gMaterial);

	for (PxU32 i = 0; i < size; i++)
	{
		for (PxU32 j = 0; j < size - i; j++)
		{
			PxTransform localTm(PxVec3(PxReal(j * 2) - PxReal(size - i), PxReal(i * 2 + 1), 0) * harfExtent);
			PxRigidDynamic* body = gPhysics->createRigidDynamic(t.transform(localTm));
			body->attachShape(*shape);
			PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
			gScene->addActor(*body);
		}
	}
	shape->release();
}

void InitPhysics(bool)
{
	gContactPositions.resize(maxCount);
	gContactImpulses.resize(maxCount);
	gContactVertices.resize(maxCount * 2);

	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* pvdTransport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*pvdTransport, PxPvdInstrumentationFlag::eALL);
	
	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(),true, gPvd);
	
	//Physx 확장 라이브러리 초기화.
	PxInitExtensions(*gPhysics, gPvd);

	PxU32 numCores = SnippetUtils::getNbPhysicalCores();
	gDispatcher = PxDefaultCpuDispatcherCreate(numCores);

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.gravity = PxVec3(0, -9.8f, 0);
	sceneDesc.filterShader = ContactReportFilterShader;
	sceneDesc.simulationEventCallback = &gContactReportCallback;
	gScene = gPhysics->createScene(sceneDesc);

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if (pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
	}
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
	gScene->addActor(*groundPlane);

	const PxU32 nbStacks = 50;

	for (PxU32 i = 0; i < nbStacks; ++i)
	{
		CreateStack(PxTransform(PxVec3(0, 3.0f, 10.f - 5.f * i)), 5, 2.0f);
	}
}

void StepPhysics(bool)
{
	gSharedIndex = 0;

	gScene->simulate(1.0f / 60.0f);

#if !PARALLEL_CALLBACKS
	gScene->fetchResults(true);
#else
	// fetchResultStart를 호출하여 페어 헤더들을 받아온다.
	const PxContactPairHeader* pairHeaders=nullptr;
	PxU32 numContactPairs = 0;
	gScene->fetchResultsStart(
		pairHeaders, //out
		numContactPairs, //out
		true // true일 경우 결과를 받아올때까지 대기한다.
	);

	// 콜백이 병렬로 처리 된 후 continuation task가 실행되도록 설정
	gCallbackFinishTask.setContinuation(*gScene->getTaskManager(), nullptr);
	gCallbackFinishTask.Reset();

	//콜백 작업
	gScene->processCallbacks(&gCallbackFinishTask);

	gCallbackFinishTask.removeReference();
	gCallbackFinishTask.Wait();
	
	gScene->fetchResultsFinish();
#endif

	printf("%d contact reports\n", PxU32(gSharedIndex));
}


void CleanupPhysics(bool /*interactive*/)
{
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PxCloseExtensions();
	PX_RELEASE(gPhysics);
	if (gPvd)
	{
		PxPvdTransport* transport = gPvd->getTransport();
		gPvd->release();	gPvd = NULL;
		PX_RELEASE(transport);
	}
	PX_RELEASE(gFoundation);

	printf("SnippetSplitFetchResults done.\n");
}

int SnippetMain(int, const char* const*)
{
#ifdef RENDER_SNIPPET
	extern void RenderLoop();
	RenderLoop();
#else
	InitPhysics(false);
	for (PxU32 i = 0; i < 250; i++)
		StepPhysics(false);
	CleanupPhysics(false);
#endif

	return 0;
}
