
#include <ctype.h>
#include <Windows.h>

#include "PxPhysicsAPI.h"

#include "SnippetPrint.h"
#include "SnippetPVD.h"
#include "SnippetUtils.h"

using namespace physx;

PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;

PxFoundation* gFoundation = NULL;
PxPhysics* gPhysics = NULL;

PxDefaultCpuDispatcher* gDispatcher = NULL;
PxScene* gScene = NULL;

PxMaterial* gMaterial = NULL;

PxPvd* gPvd = NULL;

//슈팅 볼 생성
PxRigidDynamic* CreateDynamic(const PxTransform& t, const PxGeometry& geometry,
    const PxVec3& velocity = PxVec3(0))
{
    PxRigidDynamic* ball = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 10.0f);
    ball->setLinearVelocity(velocity);
    gScene->addActor(*ball);

    return ball;
}

// 회전제한을 가진 조인트 생성
PxJoint* CreateLimitedSpherical(PxRigidActor* a0, const PxTransform& t0,
    PxRigidActor* a1, const PxTransform& t1)
{
    PxSphericalJoint* joint = PxSphericalJointCreate(*gPhysics, a0, t0, a1, t1);
    //콘 형태로 회전 반경을 제한
    joint->setLimitCone(PxJointLimitCone(PxPi / 4, PxPi / 4, 0.05f));
    //회전 반경 제한 옵션을 킨다.. SphericalJoint 전용 플래그이므로 주의할 것.
    joint->setSphericalJointFlag(PxSphericalJointFlag::eLIMIT_ENABLED, true);
    return joint;
}

// 고정형, 브레이커블 조인트 생성
PxJoint* CreateBreakableFixed(PxRigidActor* a0, const PxTransform& t0,
    PxRigidActor* a1, const PxTransform& t1)
{
    PxFixedJoint* joint = PxFixedJointCreate(*gPhysics, a0, t0, a1, t1);
    joint->setBreakForce(1000, 10000);
    //구동 강도의 한계를 impulses에서 Force로 변경
    joint->setConstraintFlag(PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES, true);
    joint->setConstraintFlag(PxConstraintFlag::eDISABLE_PREPROCESSING, true);
  
    return joint;
}

// 위치를 유지하고 각종 제한을 둘 수 있는 D6 Joint 생성.
// https://gameworksdocs.nvidia.com/PhysX/4.1/documentation/physxguide/Manual/Joints.html#d6-joint
PxJoint* CreateDampedD6(PxRigidActor* a0, const PxTransform& t0,
    PxRigidActor* a1, const PxTransform& t1)
{
    PxD6Joint* joint = PxD6JointCreate(*gPhysics, a0, t0, a1, t1);
    joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);  //X축 동작을 자유롭게 풀어준다.
    joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE); //Y축 동작을 자유롭게 풀어준다.
    joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE); //Z축 동작을 자유롭게 풀어준다.
    joint->setDrive(PxD6Drive::eSLERP, PxD6JointDrive(0, 1000, FLT_MAX, true)); //SLerp경로를 따라서 3축으로 회전.

    return joint;
}

typedef PxJoint* (*JointCreateFunction)(PxRigidActor* a0, const PxTransform& t0, PxRigidActor* a1, const PxTransform& t1);

void CreateChain(const PxTransform& t, PxU32 length, const PxGeometry& g,
    PxReal separation, JointCreateFunction createJoint)
{
    PxVec3 offset(separation/2, 0, 0);
    PxTransform localTm(offset);
    PxRigidBody* prev = nullptr;

    for (PxU32 i = 0; i < length; i++)
    {
        PxRigidBody* currRigid = PxCreateDynamic(*gPhysics, t * localTm, g, *gMaterial, 1.0f);
        (*createJoint)(prev, prev ? PxTransform(offset) : t, currRigid, PxTransform(-offset));
        gScene->addActor(*currRigid);
        prev = currRigid;
        localTm.p.x += separation;
    }
}

void InitPhysics(bool)
{
    gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

    gPvd = PxCreatePvd(*gFoundation);
    PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
    gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

    SYSTEM_INFO info;
    GetSystemInfo(&info);

    gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);
    gDispatcher = PxDefaultCpuDispatcherCreate(info.dwNumberOfProcessors);

    PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
    sceneDesc.gravity = PxVec3(0, 9.8f, 0);
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
    PxRigidStatic* ground = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);

    gScene->addActor(*ground);
    PxBoxGeometry barGeometry(2.0f, 0.5f, 0.5f);
    CreateChain(PxTransform(PxVec3(0.0f, 20.0f, 0.0f)), 5, barGeometry, 4.0f, CreateLimitedSpherical);
    CreateChain(PxTransform(PxVec3(0.0f, 20.0f, -10.0f)), 5, barGeometry, 4.0f, CreateBreakableFixed);
    CreateChain(PxTransform(PxVec3(0.0f, 20.0f, -20.0f)), 5, barGeometry, 4.0f, CreateDampedD6);
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
    PxCloseExtensions();
    PX_RELEASE(gPhysics);
    if (gPvd)
    {
        PxPvdTransport* transport = gPvd->getTransport();
        gPvd->release();	gPvd = NULL;
        PX_RELEASE(transport);
    }
    PX_RELEASE(gFoundation);

    printf("SnippetJoint done.\n");
}

void KeyPress(unsigned char key, const PxTransform& camera)
{
    switch (toupper(key))
    {
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
    InitPhysics(false);
    for (PxU32 i = 0; i < frameCount; i++)
        StepPhysics(false);
    CleanupPhysics(false);
#endif

    return 0;
}