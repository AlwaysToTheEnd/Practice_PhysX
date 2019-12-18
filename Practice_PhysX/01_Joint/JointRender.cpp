#ifdef RENDER_SNIPPET

#include <vector>

#include "PxPhysicsAPI.h"

#include "SnippetRender.h"
#include "SnippetCamera.h"

using namespace physx;

extern void InitPhysics(bool interactive);
extern void StepPhysics(bool interactive);
extern void CleanupPhysics(bool interactive);
extern void KeyPress(unsigned char key, const PxTransform& camera);

namespace
{
	Snippets::Camera* sCamera;

	void MotionCallback(int x, int y)
	{
		sCamera->handleMotion(x, y);
	}

	void KeyboardCallback(unsigned char key, int x, int y)
	{
		if (key == 27)
			exit(0);

		if (!sCamera->handleKey(key, x, y))
			KeyPress(key, sCamera->getTransform());
	}

	void MouseCallback(int button, int state, int x, int y)
	{
		sCamera->handleMouse(button, state, x, y);
	}

	void IdleCallback()
	{
		glutPostRedisplay();
	}

	void RenderCallback()
	{
		StepPhysics(true);

		Snippets::startRender(sCamera->getEye(), sCamera->getDir());

		PxScene* scene;
		PxGetPhysics().getScenes(&scene, 1);
		PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
		if (nbActors)
		{
			std::vector<PxRigidActor*> actors(nbActors);
			scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&actors[0]), nbActors);
			Snippets::renderActors(&actors[0], static_cast<PxU32>(actors.size()), true);
		}

		Snippets::finishRender();
	}

	void ExitCallback(void)
	{
		delete sCamera;
		CleanupPhysics(true);
	}
}

void RenderLoop()
{
	sCamera = new Snippets::Camera(PxVec3(50.0f, 50.0f, 50.0f), PxVec3(-0.6f, -0.2f, -0.7f));

	Snippets::setupDefaultWindow("PhysX Snippet Joint");
	Snippets::setupDefaultRenderState();

	glutIdleFunc(IdleCallback);
	glutDisplayFunc(RenderCallback);
	glutKeyboardFunc(KeyboardCallback);
	glutMouseFunc(MouseCallback);
	glutMotionFunc(MotionCallback);
	MotionCallback(0, 0);

	atexit(ExitCallback);

	InitPhysics(true);
	glutMainLoop();
}
#endif
